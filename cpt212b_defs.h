 /**
 * @file cpt212b_defs.c
 *
 * @author Alexander Grin
 * @copyright (C) 2020 Grin development. All rights reserved.
 */

#ifndef CPT212B_DEFS_H
#define CPT212B_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/*************************** C types headers *****************************/
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif
/*************************** Common macros   *****************************/
/** C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL                                (0)
#else
#define NULL                                ((void *) 0)
#endif
#endif

/** Define Endian-ness */
#ifndef __ORDER_LITTLE_ENDIAN__
#define __ORDER_LITTLE_ENDIAN__             (0)
#endif

#ifndef __BYTE_ORDER__
#define __BYTE_ORDER__                      __ORDER_LITTLE_ENDIAN__
#endif

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN                       (1)
#endif
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#ifndef BIG_ENDIAN
#define BIG_ENDIAN                          (1)
#endif
#endif

#if ((BIG_ENDIAN) || (defined __C51__) || (defined __RC51__) || (defined _CC51))
#define htobe16(x) (x)

//! Little Endian Compilers: Bit swapping necessary
#elif ((LITTLE_ENDIAN) || (defined __GNUC__) || (defined __clang__) ||(defined SDCC) || (defined HI_TECH_C) || (defined __ICC8051__))
#define htobe16(x) (((x) >> 8) | ((x) << 8))
#else
#error "Compiler not defined. Endian-ness of the compiler cannot be determined, and is necessary for CPT2xxx device configuration."
#endif

/*************************** Type definition *****************************/
//! I2C slave address
#define CPT212B_I2C_ADDR_DEFAULT            (0xC0)
#define CPT212B_I2C_ADDR_PROGRAMMED         (0xE0)

/**
 * @brief CPT212B default configuration profile definition
 *
 * @note This structure must be transmitted to the CPT device in Big-endian format!
 * @details User is able to create default configuration profile with GUI by
 *          Simplicity Studio Xpress Configurator
 */
#define CPT212B_DEFAULT_CONFIG_PROFILE \
{ \
    {0x00, 0x00, 0x00},                                                         /* reserved_0 */ \
    {0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C},   /* Active Threshold */ \
    {0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28},   /* Inactive Threshold */ \
    {0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B},   /* Average Touch Delta */ \
    {0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02},   /* Accumulation */ \
    {0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06},   /* Gain */ \
    htobe16(0x0014),                                                            /* active mode period */ \
    htobe16(0x00C8),                                                            /* sleep mode period */ \
    0x02,                                                                       /* button debounce */ \
    {0xFF, 0x07},                                                               /* active mode mask */ \
    {0xFF, 0x07},                                                               /* sleep mode mask */ \
    0x64,                                                                       /* counts before sleep */ \
    0x01,                                                                       /* active mode scan type */ \
    {0x00, 0x00, 0x00, 0x00},                                                   /* reserved_1 */ \
    0x10,                                                                       /* buzzer output duration */ \
    0x64,                                                                       /* buzzer output frequency */ \
    0x01,                                                                       /* buzzer drive strength */ \
    0x00,                                                                       /* touch time duration */ \
    0x00,                                                                       /* mutex buttons */ \
    0x00,                                                                       /* prox settings */ \
    0xE0,                                                                       /* I2C slave address */ \
    0x00,                                                                       /* reserved_2 */ \
    0x01,                                                                       /* I2C timeout duration */ \
    {'C', 'S', '0', '0', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'},       /* CS00 sensor string */ \
    {'C', 'S', '0', '1', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'},       /* CS01 sensor string */ \
    {'C', 'S', '0', '2', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'},       /* CS02 sensor string */ \
    {'C', 'S', '0', '3', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'},       /* CS03 sensor string */ \
    {'C', 'S', '0', '4', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'},       /* CS04 sensor string */ \
    {'C', 'S', '0', '5', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'},       /* CS05 sensor string */ \
    {'C', 'S', '0', '6', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'},       /* CS06 sensor string */ \
    {'C', 'S', '0', '7', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'},       /* CS07 sensor string */ \
    {'C', 'S', '0', '8', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'},       /* CS08 sensor string */ \
    {'C', 'S', '0', '9', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'},       /* CS09 sensor string */ \
    {'C', 'S', '1', '0', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'},       /* CS10 sensor string */ \
    {'C', 'S', '1', '1', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'},       /* CS11 sensor string */ \
}

/**
 * @brief Drivers error codes enumerations
 */
typedef enum {
    CPT212B_OK = 0,             //!< CPT212B_OK
    CPT212B_E_FAILED = -1,      //!< CPT212B_E_FAILED
    CPT212B_E_NULL_PTR = -2,    //!< CPT212B_E_NULL_PTR
    CPT212B_E_I2C_COM_FAIL = -3,//!< CPT212B_E_I2C_COM_FAIL
    CPT212B_E_PACKET_LOST = -4, //!< CPT212B_E_PACKET_LOST
    CPT212B_E_PACKET_IVALID = -5//!< CPT212B_E_PACKET_IVALID
} cpt212b_err_t;

/**
 * @brief Configuration profile structure
 * @note This structure must be transmitted to the CPT device in Big-endian format!
 * @details Size of profile must be 207 bytes, otherwise, apply forced
 *          packaging methods of compiler.
 */
#pragma pack(1)
struct cpt212b_config_profile
{                                           //  Min-Max |  Default
    uint8_t reserved_0[3];                  //      0-0 |       0
    uint8_t active_threshold[12];           //    1-100 |      60
    uint8_t inactive_threshold[12];         //    1-100 |      40
    uint8_t average_touch_delta[12];        //    1-255 |      43
    uint8_t accumulation[12];               //      0-5 |       2
    uint8_t gain[12];                       //      0-7 |       6
    uint16_t active_mode_period;            //   1-5000 |      20
    uint16_t sleep_mode_period;             //   1-5000 |     200
    uint8_t button_debounce;                //     1-15 |       2
    uint8_t active_mode_mask[2];            //      0-1 |       1
    uint8_t sleep_mode_mask[2];             //      0-1 |       1
    uint8_t counts_before_sleep;            //    2-255 |     100
    uint8_t active_mode_scan_type;          //      0-1 |       1
    uint8_t reserved_1[4];                  //      0-0 |       0
    uint8_t buzzer_output_duration;         //    0-255 |      16
    uint8_t buzzer_output_frequency;        //    0-100 |     100
    uint8_t buzzer_drive_strength;          //      0-1 |       1
    uint8_t touch_time_duration;            //     0-14 |       0
    uint8_t mutex_buttons;                  //      0-1 |       0
    uint8_t prox_settings;                  //     0-15 |       0
    uint8_t i2c_slave_address;              //    0-255 |     224
    uint8_t reserved_2;                     //      0-0 |       0
    uint8_t i2c_timeout_duration;           //     0-15 |       1
    uint8_t cs00_sensor_string[10];         //          |  "CS00"
    uint8_t cs01_sensor_string[10];         //          |  "CS01"
    uint8_t cs02_sensor_string[10];         //          |  "CS02"
    uint8_t cs03_sensor_string[10];         //          |  "CS03"
    uint8_t cs04_sensor_string[10];         //          |  "CS04"
    uint8_t cs05_sensor_string[10];         //          |  "CS05"
    uint8_t cs06_sensor_string[10];         //          |  "CS06"
    uint8_t cs07_sensor_string[10];         //          |  "CS07"
    uint8_t cs08_sensor_string[10];         //          |  "CS08"
    uint8_t cs09_sensor_string[10];         //          |  "CS09"
    uint8_t cs10_sensor_string[10];         //          |  "CS10"
    uint8_t cs11_sensor_string[10];         //          |  "CS11"
};
#pragma pack()

/**
 * @brief Type of configuration profile structure
 */
typedef struct cpt212b_config_profile cpt212b_config_profile_t;

/**
 * @brief Enumeration of GPIO pins
 */
typedef enum {
    CPT212B_RESET_PIN,//!< CPT212B_RESET_PIN
    CPT212B_INT_PIN   //!< CPT212B_INT_PIN
} cpt212b_pin_t;

/**
 * @brief GPIO output pin state
 */
typedef enum {
   CPT212B_PIN_LOW = 0,//!< CPT212B_PIN_LOW
   CPT212B_PIN_HIGH    //!< CPT212B_PIN_HIGH
} cpt212b_pin_state_t;

/**
 * @brief I2C communication function pointer
 *
 * @param dev_addr - device address
 * @param data - pointer to the data buffer
 * @param len - data buffer length
 * @return CPT212B_OK - communication success
 *         other - error code from cpt212b_err_t enumeration
 */
typedef int8_t (*cpt212b_i2c_com_fptr_t)(uint8_t dev_addr, uint8_t *data,
        uint16_t len);

/**
 * @brief Set GPIO pin output state function pointer
 *
 * @param pin - GPIO pin
 * @param state - pin state
 */
typedef void (*cpt212b_pin_output_set_fptr_t)(cpt212b_pin_t pin,
        cpt212b_pin_state_t state);

/**
 * @brief Delay function pointer
 * @param period_ms - milliseconds period
 */
typedef void (*cpt212b_delay_fptr_t)(uint32_t period_ms);

typedef struct {
    cpt212b_i2c_com_fptr_t i2c_read;
    cpt212b_i2c_com_fptr_t i2c_write;
    cpt212b_pin_output_set_fptr_t pin_output_set;
    cpt212b_delay_fptr_t delay_ms;
} cpt212b_intrf_t;

typedef enum {
    CPT212B_TOUCH_EVENT = 0x0,
    CPT212B_RELEASE_EVENT,
    CPT212B_PROXIMITY_EVENT,
    CPT212B_EVENT_MAX
} cpt212b_event_type_t;

typedef enum {
    CPT212B_CS_INDEX_0 = 0x0,
    CPT212B_CS_INDEX_1,
    CPT212B_CS_INDEX_2,
    CPT212B_CS_INDEX_3,
    CPT212B_CS_INDEX_4,
    CPT212B_CS_INDEX_5,
    CPT212B_CS_INDEX_6,
    CPT212B_CS_INDEX_7,
    CPT212B_CS_INDEX_8,
    CPT212B_CS_INDEX_9,
    CPT212B_CS_INDEX_10,
    CPT212B_CS_INDEX_11,
    CPT212B_CS_INDEX_MAX,
} cpt212b_cs_index_t;

/**
 * @brief CPT212B packet structure
 */
typedef struct
{
  uint8_t packet_counter :4;
  cpt212b_event_type_t event_type :4;
  cpt212b_cs_index_t cs_index;
  uint8_t reserved;
} cpt212b_packet_t;

/**
 * @brief CPT212B device structure
 */
typedef struct {
    const cpt212b_intrf_t intrface;
    uint8_t i2c_programmed_addr;
    cpt212b_packet_t last_packet;
} cpt212b_dev_t;

#ifdef __cplusplus
}
#endif

#endif //!CPT212B_DEFS_H
