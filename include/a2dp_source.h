/******************************************************************************
 * (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 ******************************************************************************/
/******************************************************************************
 * File Name: a2dp_source.h
 *
 * Description: Definitions for constants used in the a2dp source
 * application and function prototypes.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/
#ifndef A2DP_SOURCE_H
#define A2DP_SOURCE_H

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include "app_bt_utils/app_bt_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <pthread.h>
#include <semaphore.h>
#include <ctype.h>
#include <string.h>
#include <sys/time.h>
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_types.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"
#include "wiced_bt_a2dp_source.h"
#include "wiced_bt_a2dp_defs.h"
#include "wiced_bt_trace.h"
#include "hcidefs.h"
#include "wiced_hal_nvram.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "wiced_result.h"
#include "wiced_timer.h"

/******************************************************************************
 *                                MACROS
 *****************************************************************************/
#define A2DP_SOURCE_SDP_DB_SIZE 63

/*A2DP Sampling Frequencies */
#define AUDIO_SF_48K               1
#define AUDIO_SF_44_1K             2
#define AUDIO_SF_32K               3
#define AUDIO_SF_16K               4
/* Channel Configurations */
#define AUDIO_CHCFG_MONO           0
#define AUDIO_CHCFG_STEREO         1
#define AUDIO_CHCFG_JOINT          2
/* Recommended Max Bit Pool for High Quality Audio Joint Stereo Mode */
#define BT_AUDIO_A2DP_SBC_MAX_BITPOOL_44K   53
#define BT_AUDIO_A2DP_SBC_MAX_BITPOOL_48K   51
/* Return Codes */
#define A2DP_APP_SUCCESS           0
#define A2DP_APP_FAILED            1

#define WICED_PIN_CODE_LEN              (4U)
/* Size of the buffer used for holding the peer device key info */
#define KEY_INFO_POOL_BUFFER_SIZE       (145U)
/* Correspond to the number of peer devices */
#define KEY_INFO_POOL_BUFFER_COUNT      (2U)


/******************************************************************************
 *                         STRUCTURES AND ENUMERATIONS
 ******************************************************************************/
typedef enum
{
    AV_STATE_IDLE,              /* Initial state (channel is unused) */
    AV_STATE_CONFIGURE,         /* Remote has sent configuration request */
    AV_STATE_OPEN,              /* Data channel connected but not streaming */
    AV_STATE_STARTED,           /* Data streaming */
    AV_STATE_RECONFIG,          /* Reconfiguring stream */
    AV_STATE_DISCONNECTING      /* Disconnecting */
} AV_STATE;


typedef struct
{
    wiced_bt_device_address_t peer_bda;      /* Peer bd address */
    AV_STATE state;                          /* AVDT State machine state */
    wiced_bt_a2dp_codec_info_t codec_config; /*Codec config */
    uint16_t stream_mtu;                     /* MTU of stream */
    uint16_t lcid;                           /* Local identifier */
} tAV_APP_CB;

/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/
uint16_t a2dp_source_write_nvram (int nvram_id, int data_len, void *p_data);
uint16_t a2dp_source_read_nvram (int nvram_id, void *p_data, int data_len);
wiced_result_t a2dp_source_management_callback (
        wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data);

void av_app_init (void);
void a2dp_source_control_cback (wiced_bt_a2dp_source_event_t event,
                wiced_bt_a2dp_source_event_data_t *p_data);
void a2dp_source_bt_set_pairability ( uint8_t pairing_allowed );
void a2dp_source_bt_print_local_bda( void );
void a2dp_source_bt_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data );
wiced_result_t a2dp_source_bt_inquiry( uint8_t enable );
wiced_result_t a2dp_source_bt_set_visibility( uint8_t discoverability, uint8_t connectability );
wiced_result_t a2dp_source_command_connect(wiced_bt_device_address_t bd_addr, uint32_t len);
wiced_result_t a2dp_source_command_disconnect();
void a2dp_source_command_stream_config(uint8_t sf, uint8_t chcfg);
wiced_result_t a2dp_source_command_stream_start();
wiced_result_t a2dp_source_command_stream_stop();

#endif  /* A2DP_SOURCE_H */

