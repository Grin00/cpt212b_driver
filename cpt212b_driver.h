/**
 * @file cpt212b_driver.c
 *
 * @author Alexander Grin
 * @copyright (C) 2020 Grin development. All rights reserved.
 */

#ifndef CPT212B_DRIVER_H
#define CPT212B_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cpt212b_defs.h"

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
        cpt212b_config_profile_t *config);

/**
 * @brief Send a mode switch command to enter sensing mode
 * @param device - pointer to device structure
 * @return CPT212B_OK - success
 *         other - error from cpt212b_err_t enumeration
 */
cpt212b_err_t cpt212b_enter_sensing_mode(cpt212b_dev_t *device);

/**
 * @brief Get touch controller packet
 * @param device - pointer to device structure
 * @param packet - pointer to returned packet structure
 * @return CPT212B_OK - success
 *         other - error from cpt212b_err_t enumeration
 */
cpt212b_err_t cpt212b_get_packet(cpt212b_dev_t *device,
        cpt212b_packet_t *packet);

#ifdef __cplusplus
}
#endif

#endif //!CPT212B_DRIVER_H
