/**
* @file cpt212b_driver.c
* @author Alexander Grin
* @copyright (c) 2020 Grin development. All rights reserved.
*/

#include "cpt212b_driver.h"
#include <string.h>
#include <stdbool.h>

#define CPT212B_PACKET_SIZE                 (0x03)
#define CPT212B_MAX_PACKET_COUNTER          (0x0F)

//! CPT212B commands defines
#define CPT212B_MODE_SELECTION_CMD          {0x08, 0x01}
#define CPT212B_CONFIG_UNLOCK_CMD           {0x09, 0xA5, 0xF1}
#define CPT212B_CONFIG_ERASE_CMD            {0x0A}
#define CPT212B_CONFIG_WRITE_CMD            (0x0B)
#define CPT212B_CRC_WRITE_CMD               (0x0C)

//! Time for each configuration upload state
#define CPT212B_RESET_TIME_MS               (5)
#define CPT212B_BOOT_TIME_MS                (25)
#define CPT212B_UNLOCK_DELAY_MS             (50)
#define CPT212B_ERASE_DELAY_MS              (50)
#define CPT212B_WRITE_DELAY_MS              (10)
#define CPT212B_WRITE_CRC_MS                (50)

#define CPT212B_CRC_POLY                    (0x1021)

#define CPT212B_GET_CRC_MSB(x)              (x >> 8)
#define CPT212B_GET_CRC_LSB(x)              (x & 0x00FF)
#define CPT212B_GET_PACKET_COUNTER(x)       (x >> 4)
#define CPT212B_GET_EVENT_TYPE(x)           (x & 0x0F)

#define CPT212B_CONFIG_VALID                (0x80)

enum cpt212b_packet_byte {
    CPT212B_PACKET_COUNTER_BYTE = 0x0,
    CPT212B_EVENT_TYPE_BYTE = 0x0,
    CPT212B_CS_INDEX_BYTE = 0x01,
    CPT212B_RESERVED_BYTE = 0x02
};

static uint16_t cpt212b_generate_CRC(cpt212b_config_profile_t *config)
{
    uint16_t byte_index;
    uint8_t i, CRC_input;
    uint16_t CRC_acc = 0xFFFF;
    uint8_t * bytePtr = (uint8_t*)config;

    for(byte_index = 0; byte_index < 510; byte_index++) {
        //! If not all bytes of config profile have been used to generate
        //! the CRC, read another byte
        if(byte_index < sizeof(cpt212b_config_profile_t)) {
            CRC_input = *bytePtr; //! Read byte from config profile
            bytePtr++; //! Increment pointer by one byte
        }
        //! else if all bytes have been used, use padding byte of 0xFF
        else {
            CRC_input = 0xFF;
        }
        //! Create the CRC "dividend" for polynomial arithmetic
        //! (binary arithmetic with no carries)
        CRC_acc = CRC_acc ^ (CRC_input << 8);
        //! "Divide" the poly into the dividend using CRC XOR subtraction
        //! CRC_acc holds the "remainder" of each divide
        //! Only complete this division for 8 bits since input is 1 byte
        for(i = 0; i < 8; i++) {
            //! Check if the MSB is set (if MSB is 1, then the POLY can "divide"
            //! into the "dividend")
            if((CRC_acc & 0x8000) == 0x8000) {
                //! if so, shift the CRC value, and XOR "subtract" the poly
                CRC_acc = CRC_acc << 1;
                CRC_acc ^= CPT212B_CRC_POLY;
            }
            else {
                //! if not, just shift the CRC value
                CRC_acc = CRC_acc << 1;
            }
        }
    }

  return CRC_acc;
}

/**
 * @brief Check the platform interface for a null pointer
 *
 * @param intrf - pointer to interface structure
 * @return CPT212B_OK - valid
 *         CPT212B_E_NULL_PTR - interface has a null pointer function
 */
static cpt212b_err_t cpt212b_null_ptr_check(const cpt212b_intrf_t *intrf)
{
    cpt212b_err_t rslt;

    if(intrf->i2c_read == NULL ||
            intrf->i2c_write == NULL ||
            intrf->pin_output_set == NULL ||
            intrf->delay_ms == NULL) {
        rslt = CPT212B_E_NULL_PTR;
    }
    else {
        rslt = CPT212B_OK;
    }

    return rslt;
}

/**
 * @brief Device hardware reset
 *
 * @param intrf - pointer to interface pointer structure
 * @return CPT212B_OK - success
 *         other - error from cpt212b_err_t enumeration
 */
static cpt212b_err_t cpt212b_reset(const cpt212b_intrf_t *intrf)
{
    cpt212b_err_t rslt;

    rslt = cpt212b_null_ptr_check(intrf);
    if (rslt != CPT212B_OK)
        goto exit;

    intrf->pin_output_set(CPT212B_RESET_PIN, CPT212B_PIN_LOW);
    intrf->delay_ms(CPT212B_RESET_TIME_MS);
    intrf->pin_output_set(CPT212B_RESET_PIN, CPT212B_PIN_HIGH);

    intrf->delay_ms(CPT212B_BOOT_TIME_MS);

    exit: return rslt;
}

/**
 * @brief Send the configuration loading unlock command
 *
 * @param intrf - pointer to interface pointer structure
 * @return CPT212B_OK - success
 *         other - error from cpt212b_err_t enumeration
 */
static cpt212b_err_t cpt212b_config_unlock(const cpt212b_intrf_t *intrf)
{
    cpt212b_err_t rslt;
    uint8_t write_buf[] = CPT212B_CONFIG_UNLOCK_CMD;

    rslt = cpt212b_null_ptr_check(intrf);
    if(rslt != CPT212B_OK)
        goto exit;

    intrf->pin_output_set(CPT212B_INT_PIN, CPT212B_PIN_LOW);
    rslt = intrf->i2c_write(CPT212B_I2C_ADDR_DEFAULT, write_buf,
            sizeof(write_buf));
    intrf->pin_output_set(CPT212B_INT_PIN, CPT212B_PIN_HIGH);
    if(rslt != CPT212B_OK) {
        rslt = CPT212B_E_I2C_COM_FAIL;
        goto exit;
    }

    intrf->delay_ms(CPT212B_UNLOCK_DELAY_MS);

    exit: return rslt;
}

/**
 * @brief Send the configuration erase command
 *
 * @param intrf - pointer to interface pointer structure
 * @return CPT212B_OK - success
 *         other - error from cpt212b_err_t enumeration
 */
static cpt212b_err_t cpt212b_config_erase(const cpt212b_intrf_t *intrf)
{
    cpt212b_err_t rslt;
    uint8_t write_buf[] = CPT212B_CONFIG_ERASE_CMD;

    rslt = cpt212b_null_ptr_check(intrf);
    if(rslt != CPT212B_OK)
      goto exit;

    intrf->pin_output_set(CPT212B_INT_PIN, CPT212B_PIN_LOW);
    rslt = intrf->i2c_write(CPT212B_I2C_ADDR_DEFAULT, write_buf,
            sizeof(write_buf));
    intrf->pin_output_set(CPT212B_INT_PIN, CPT212B_PIN_HIGH);
    if(rslt != CPT212B_OK) {
        rslt = CPT212B_E_I2C_COM_FAIL;
        goto exit;
    }

    intrf->delay_ms(CPT212B_ERASE_DELAY_MS);

    exit: return rslt;
}

/**
 * @brief Send the configuration profile
 *
 * @param intrf - pointer to interface pointer structure
 * @param cpt212b_config - pointer to configuration profile
 * @return CPT212B_OK - success
 *         other - error from cpt212b_err_t enumeration
 */
static cpt212b_err_t cpt212b_config_write(const cpt212b_intrf_t *intrf,
        cpt212b_config_profile_t *cpt212b_config)
{
    cpt212b_err_t rslt;
    uint8_t *ptr_payload = NULL;
    const uint8_t payload_size = 8;
    uint8_t write_buf[9] = { 0 }; //! Write buffer consist of write command and
                                  //! 8 bytes payload

    rslt = cpt212b_null_ptr_check(intrf);
    if(rslt != CPT212B_OK)
      goto exit;

    if(cpt212b_config == NULL) {
        rslt = CPT212B_E_NULL_PTR;
        goto exit;
    }
    else {
        ptr_payload = (uint8_t*)cpt212b_config;
    }

    for(int i = 0; i < sizeof(cpt212b_config_profile_t); i += payload_size) {
        /**
         * The host should pad the last command up to a payload of 8 bytes,
         * with 0xFF used as padding
         */
        memset(write_buf, 0xFF, sizeof(write_buf));

        //! Initialize first byte with write command
        write_buf[0] = CPT212B_CONFIG_WRITE_CMD;

        //! Copy payload
        if(sizeof(cpt212b_config_profile_t) - i >= payload_size) {
            memcpy(&write_buf[1], &ptr_payload[i], payload_size);
        }
        else {
            memcpy(&write_buf[1], &ptr_payload[i],
                    sizeof(cpt212b_config_profile_t) - i);
        }

        intrf->pin_output_set(CPT212B_INT_PIN, CPT212B_PIN_LOW);
        rslt = intrf->i2c_write(CPT212B_I2C_ADDR_DEFAULT, write_buf,
                sizeof(write_buf));
        intrf->pin_output_set(CPT212B_INT_PIN, CPT212B_PIN_HIGH);
        if(rslt != CPT212B_OK) {
            rslt = CPT212B_E_I2C_COM_FAIL;
            goto exit;
        }

        intrf->delay_ms(CPT212B_WRITE_DELAY_MS);
    }

    exit: return rslt;
}

/**
 * @brief Send the configuration erase command
 *
 * @param intrf - pointer to interface pointer structure
 * @param crc_acc - configuration profile CRC
 * @return CPT212B_OK - success
 *         other - error from cpt212b_err_t enumeration
 */
static cpt212b_err_t cpt212b_crc_write(const cpt212b_intrf_t *intrf,
        uint16_t crc_acc)
{
    cpt212b_err_t rslt;
    uint8_t crc_msb = CPT212B_GET_CRC_MSB(crc_acc);
    uint8_t crc_lsb = CPT212B_GET_CRC_LSB(crc_acc);
    uint8_t write_buf[3] = {0};//! Write buffer consist of CRC write command and
                               //! 2 bytes CRC
    write_buf[0] = CPT212B_CRC_WRITE_CMD;
    write_buf[1] = crc_msb;
    write_buf[2] = crc_lsb;

    rslt = cpt212b_null_ptr_check(intrf);
    if(rslt != CPT212B_OK)
      goto exit;

    intrf->pin_output_set(CPT212B_INT_PIN, CPT212B_PIN_LOW);
    rslt = intrf->i2c_write(CPT212B_I2C_ADDR_DEFAULT, write_buf,
            sizeof(write_buf));
    intrf->pin_output_set(CPT212B_INT_PIN, CPT212B_PIN_HIGH);
    if(rslt != CPT212B_OK) {
        rslt = CPT212B_E_I2C_COM_FAIL;
        goto exit;
    }

    intrf->delay_ms(CPT212B_WRITE_CRC_MS);

    exit: return rslt;
}

/**
 * @brief Check configuration profile validity
 * @note result = 1 - configuration profile valid
 *       result = 0 - configuration profile invalid
 *
 * @param intrf - pointer to interface pointer structure
 * @param result - configuration profile state
 * @return CPT212B_OK - success
 *         other - error from cpt212b_err_t enumeration
 */
static cpt212b_err_t cpt212b_config_is_valid(const cpt212b_intrf_t *intrf,
        uint8_t *result)
{
    cpt212b_err_t rslt;
    uint8_t read_buf[1];

    rslt = cpt212b_null_ptr_check(intrf);
    if(rslt != CPT212B_OK)
      goto exit;

    if(result == NULL) {
        rslt = CPT212B_E_NULL_PTR;
        goto exit;
    }

    intrf->pin_output_set(CPT212B_INT_PIN, CPT212B_PIN_LOW);
    rslt = intrf->i2c_read(CPT212B_I2C_ADDR_DEFAULT, read_buf,
            sizeof(read_buf));
    intrf->pin_output_set(CPT212B_INT_PIN, CPT212B_PIN_HIGH);
    if(rslt != CPT212B_OK) {
        rslt = CPT212B_E_I2C_COM_FAIL;
        goto exit;
    }

  if(read_buf[0] == CPT212B_CONFIG_VALID)
    *result = 1;
  else
    *result = 0;

  exit: return rslt;
}

/**
 * @brief Send a mode switch command to enter sensing mode
 * @param device - pointer to device structure
 * @return CPT212B_OK - success
 *         other - error from cpt212b_err_t enumeration
 */
cpt212b_err_t cpt212b_enter_sensing_mode(cpt212b_dev_t *device)
{
    cpt212b_err_t rslt;
    uint8_t write_buf[] = CPT212B_MODE_SELECTION_CMD;
    const cpt212b_intrf_t *intrf = &device->intrface;

    rslt = cpt212b_null_ptr_check(intrf);
    if(rslt != CPT212B_OK)
        goto exit;

    intrf->pin_output_set(CPT212B_INT_PIN, CPT212B_PIN_LOW);
    rslt = intrf->i2c_write(CPT212B_I2C_ADDR_DEFAULT, write_buf,
            sizeof(write_buf));
    intrf->pin_output_set(CPT212B_INT_PIN, CPT212B_PIN_HIGH);

    exit: return rslt;
}

/**
 * @brief Upload configuration profile
 * @note CPT212B need time for upload and validate configuration profile,
 *       therefore, blocking delays are used here
 *
 * @param device - pointer to device structure
 * @param config - pointer to configuration profile
 * @return CPT212B_OK - success
 *         other - error from cpt212b_err_t enumeration
 */
cpt212b_err_t cpt212b_config_upload(cpt212b_dev_t *device,
        cpt212b_config_profile_t *config)
{
    cpt212b_err_t rslt;
    uint8_t config_is_valid = false;
    const cpt212b_intrf_t *intrf;

    if(device == NULL) {
        rslt = CPT212B_E_NULL_PTR;
        goto exit;
    }

    intrf = &device->intrface;
    rslt = cpt212b_null_ptr_check(intrf);
    if(rslt != CPT212B_OK)
        goto exit;

    if(config == NULL) {
        rslt = CPT212B_E_NULL_PTR;
        goto exit;
    }

    //! Generate CRC
    uint16_t crc_acc = cpt212b_generate_CRC(config);

    /**
     * The device will respond to commands with address 0xC0 if the
     * host drives the interrupt pin low, using the pin as a chip select
     * (active LOW).
     */
    intrf->pin_output_set(CPT212B_INT_PIN, CPT212B_PIN_HIGH);

    //! Reset CPT212B to boot in configuration mode
    rslt = cpt212b_reset(intrf);
    if(rslt != CPT212B_OK)
        goto exit;

    rslt = cpt212b_config_unlock(intrf);
    if(rslt != CPT212B_OK)
        goto exit;

    rslt = cpt212b_config_erase(intrf);
    if(rslt != CPT212B_OK)
        goto exit;

    rslt = cpt212b_config_write(intrf, config);
    if(rslt != CPT212B_OK)
        goto exit;

    rslt = cpt212b_crc_write(intrf, crc_acc);
    if(rslt != CPT212B_OK)
        goto exit;

    rslt = cpt212b_config_is_valid(intrf, &config_is_valid);
    if(rslt != CPT212B_OK)
        goto exit;

    if(config_is_valid == false) {
        rslt = CPT212B_E_FAILED;
        goto exit;
    }

    device->i2c_programmed_addr = config->i2c_slave_address;

    exit: return rslt;
}

/**
 * @brief Get touch controller packet
 * @param device - pointer to device structure
 * @param packet - pointer to returned packet structure
 * @return CPT212B_OK - success
 *         other - error from cpt212b_err_t enumeration
 */
cpt212b_err_t cpt212b_get_packet(cpt212b_dev_t *device, cpt212b_packet_t *packet)
{
    cpt212b_err_t rslt = CPT212B_E_FAILED;
    const cpt212b_intrf_t *intrf = NULL;
    uint8_t read_buf[CPT212B_PACKET_SIZE];
    uint8_t packet_counter = 0;
    uint8_t event_type = 0;
    uint8_t cs_index = 0;
    uint8_t reserved = 0;

    if(device == NULL) {
        rslt = CPT212B_E_NULL_PTR;
        goto exit;
    }

    intrf = &device->intrface;
    rslt = cpt212b_null_ptr_check(intrf);
    if(rslt != CPT212B_OK)
        goto exit;

    if(packet == NULL) {
        rslt = CPT212B_E_NULL_PTR;
        goto exit;
    }

    memset(read_buf, 0x00, sizeof(read_buf));

    rslt = intrf->i2c_read(device->i2c_programmed_addr, read_buf,
            sizeof(read_buf));
    if(rslt != CPT212B_OK) {
        rslt = CPT212B_E_I2C_COM_FAIL;
        goto exit;
    }

    packet_counter = CPT212B_GET_PACKET_COUNTER(
            read_buf[CPT212B_PACKET_COUNTER_BYTE]);
    event_type = CPT212B_GET_EVENT_TYPE(read_buf[CPT212B_PACKET_COUNTER_BYTE]);
    cs_index = read_buf[CPT212B_CS_INDEX_BYTE];
    reserved = read_buf[CPT212B_RESERVED_BYTE];

    if(packet_counter > CPT212B_MAX_PACKET_COUNTER) {
        rslt = CPT212B_E_PACKET_IVALID;
        goto exit;
    }
    if(event_type >= CPT212B_EVENT_MAX) {
        rslt = CPT212B_E_PACKET_IVALID;
        goto exit;
    }
    if(cs_index >= CPT212B_CS_INDEX_MAX) {
        rslt = CPT212B_E_PACKET_IVALID;
        goto exit;
    }
    if(reserved != 0x00) {
        //! Not sure that it is critical error
        rslt = CPT212B_E_PACKET_IVALID;
        goto exit;
    }

    if(packet_counter &&
            packet_counter != (device->last_packet.packet_counter + 1)) {
        rslt = CPT212B_E_PACKET_LOST;
    }

    packet->packet_counter = packet_counter;
    packet->event_type = event_type;
    packet->cs_index = cs_index;
    packet->reserved = reserved;

    //! Update last packet
    memcpy(&device->last_packet, packet, sizeof(cpt212b_packet_t));

    exit: return rslt;
}
