/*******************************************************************************
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
 *******************************************************************************/
/******************************************************************************
 * File Name: wiced_bt_cfg.c
 *
 * Description: Runtime Bluetooth stack configuration parameters.
 *
 * Related Document: See README.md
 *
 *******************************************************************************/
/******************************************************************************
 *                                INCLUDES
 ******************************************************************************/
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"

#include "a2dp_source.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_a2dp_source.h"
#include "wiced_bt_a2d_sbc.h"
#include "wiced_bt_a2dp_defs.h"

/******************************************************************************
 *                                MACROS
 ******************************************************************************/
#define WICED_DEVICE_NAME                       "IFX A2DP Source"

/******************************************************************************
 *                                GLOBAL VARIABLES
 ******************************************************************************/
/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
#if BTSTACK_VER >= 0x03000001
/* BLE SCAN Setting */
const wiced_bt_cfg_ble_scan_settings_t wiced_bt_cfg_scan_settings =
{
    .scan_mode = BTM_BLE_SCAN_MODE_ACTIVE, /**< BLE scan mode ( BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE ) */

    /* Advertisement scan configuration */
    .high_duty_scan_interval = 96, /**< High duty scan interval */
    .high_duty_scan_window = 48,   /**< High duty scan window */
    .high_duty_scan_duration = 30, /**< High duty scan duration in seconds ( 0 for infinite ) */

    .low_duty_scan_interval = 2048,/**< Low duty scan interval  */
    .low_duty_scan_window = 48,    /**< Low duty scan window */
    .low_duty_scan_duration = 30,  /**< Low duty scan duration in seconds ( 0 for infinite ) */

    /* Connection scan configuration */
    .high_duty_conn_scan_interval = 96, /**< High duty cycle connection scan interval */
    .high_duty_conn_scan_window = 48,   /**< High duty cycle connection scan window */
    .high_duty_conn_duration = 30,      /**< High duty cycle connection duration in seconds ( 0 for infinite ) */

    .low_duty_conn_scan_interval = 2048, /**< Low duty cycle connection scan interval */
    .low_duty_conn_scan_window = 48,    /**< Low duty cycle connection scan window */
    .low_duty_conn_duration = 30,       /**< Low duty cycle connection duration in seconds ( 0 for infinite ) */

    /* Connection configuration */
    .conn_min_interval = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,               /**< Minimum connection interval */
    .conn_max_interval = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,               /**< Maximum connection interval */
    .conn_latency = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                         /**< Connection latency */
    .conn_supervision_timeout = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT, /**< Connection link supervision timeout */
};

/* BLE ADV Setting */
const wiced_bt_cfg_ble_advert_settings_t wiced_bt_cfg_adv_settings =
{
    .channel_map = BTM_BLE_ADVERT_CHNL_37 | /**< Advertising channel map ( mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39 ) */
    BTM_BLE_ADVERT_CHNL_38 |
    BTM_BLE_ADVERT_CHNL_39,

    .high_duty_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,
    /**< High duty undirected connectable minimum advertising interval */
    .high_duty_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,
    /**< High duty undirected connectable maximum advertising interval */
    .high_duty_duration = 30,
    /**< High duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

    .low_duty_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,
    /**< Low duty undirected connectable minimum advertising interval */
    .low_duty_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MAX_INTERVAL,
    /**< Low duty undirected connectable maximum advertising interval */
    .low_duty_duration = 60,
    /**< Low duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

    .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,
    /**< High duty directed connectable minimum advertising interval */
    .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,
    /**< High duty directed connectable maximum advertising interval */

    .low_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,
    /**< Low duty directed connectable minimum advertising interval */
    .low_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,
    /**< Low duty directed connectable maximum advertising interval */
    .low_duty_directed_duration = 30,
    /**< Low duty directed connectable advertising duration in seconds ( 0 for infinite ) */

    .high_duty_nonconn_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,
    /**< High duty non-connectable minimum advertising interval */
    .high_duty_nonconn_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,
    /**< High duty non-connectable maximum advertising interval */
    .high_duty_nonconn_duration = 30,                                                          /**< High duty non-connectable advertising duration in seconds ( 0 for infinite ) */

    .low_duty_nonconn_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,
    /**< Low duty non-connectable minimum advertising interval */
    .low_duty_nonconn_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,
    /**< Low duty non-connectable maximum advertising interval */
    .low_duty_nonconn_duration = 0,
    /**< Low duty non-connectable advertising duration in seconds ( 0 for infinite ) */
};

/* L2CAP Setting */
const wiced_bt_cfg_l2cap_application_t wiced_bt_cfg_l2cap_app =
        /* Application managed l2cap protocol configuration */
{
    /* BR EDR l2cap configuration */
    .max_app_l2cap_psms = 0,      /**< Maximum number of application-managed BR/EDR PSMs */
    .max_app_l2cap_channels = 0, /**< Maximum number of application-managed BR/EDR channels  */

    .max_app_l2cap_br_edr_ertm_chnls = 0,  /**< Maximum ERTM channels allowed */
    .max_app_l2cap_br_edr_ertm_tx_win = 0, /**< Maximum ERTM TX Window allowed */
                            /* LE L2cap connection-oriented channels configuration */
    .max_app_l2cap_le_fixed_channels = 0,
};

/* BR Setting */
const wiced_bt_cfg_br_t wiced_bt_cfg_br =
{
    .br_max_simultaneous_links = 3,
    .br_max_rx_pdu_size = 1024,
    .device_class = {0x24, 0x04, 0x18},                     /**< Local device class */

    .rfcomm_cfg = /* RFCOMM configuration */
    {
        .max_links = 0, /**< Maximum number of simultaneous connected remote devices.
        Should be less than or equal to l2cap_application_max_links */
        .max_ports = 0, /**< Maximum number of simultaneous RFCOMM ports */
    },
    .avdt_cfg = /* Audio/Video Distribution configuration */
    {
        .max_links = 1, /**< Maximum simultaneous audio/video links */
        .max_seps = 3,  /**< Maximum number of stream end points */
    },

    .avrc_cfg = /* Audio/Video Remote Control configuration */
    {
        .max_links = 1, /**< Maximum simultaneous remote control links */
    },
};

/* ISOC Setting */
const wiced_bt_cfg_isoc_t wiced_bt_cfg_isoc =
{
    .max_cis_conn = 0,
    .max_cig_count = 0,
    .max_sdu_size = 0,
    .channel_count = 0,
    .max_buffers_per_cis = 0,
};

/* BLE Setting */
const wiced_bt_cfg_ble_t wiced_bt_cfg_ble =
{
    .ble_max_simultaneous_links = 1,
    .ble_max_rx_pdu_size = 365,
    .appearance = APPEARANCE_GENERIC_TAG,    /**< GATT appearance (see gatt_appearance_e) */
    .rpa_refresh_timeout = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,   /**< Interval of  random address refreshing - secs */
    .host_addr_resolution_db_size = 5, /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/
    .p_ble_scan_cfg = &wiced_bt_cfg_scan_settings,
    .p_ble_advert_cfg = &wiced_bt_cfg_adv_settings,
    .default_ble_power_level = 0,  /**< Default BLE Power */
};

/* GATT Setting */
const wiced_bt_cfg_gatt_t wiced_bt_cfg_gatt =
{
    .max_db_service_modules = 0,  /**< Maximum number of service modules in the DB*/
    .max_eatt_bearers = 0,        /**< Maximum number of allowed gatt bearers */
};

 /* wiced_bt core stack configuration */
const wiced_bt_cfg_settings_t a2dp_source_cfg_settings =
{
    .device_name = (uint8_t *)"IFX A2DP Source",            /**< Local device name ( NULL terminated ) */
    .security_required = BTM_SEC_BEST_EFFORT, /**< Security requirements mask */

    .p_br_cfg = &wiced_bt_cfg_br,
    .p_ble_cfg = &wiced_bt_cfg_ble,
    .p_gatt_cfg = &wiced_bt_cfg_gatt,
    .p_isoc_cfg = &wiced_bt_cfg_isoc,
    .p_l2cap_app_cfg = &wiced_bt_cfg_l2cap_app,
};

#else /* !BTSTACK_VER*/
const wiced_bt_cfg_settings_t a2dp_source_cfg_settings =
{
    .device_name                         = (uint8_t *)WICED_DEVICE_NAME,                               /**< Local device name (NULL terminated) */
    .device_class                        = {0x24, 0x04, 0x18},                                         /**< Local device class */
    .security_requirement_mask           = (  BTM_SEC_IN_AUTHENTICATE | BTM_SEC_OUT_AUTHENTICATE | BTM_SEC_ENCRYPT ), /**< Security requirements mask (BTM_SEC_NONE, or combinination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT (see #wiced_bt_sec_level_e)) */

    .max_simultaneous_links              = 3,                                                          /**< Maximum number simultaneous links to different devices */

    .br_edr_scan_cfg =                                              /* BR/EDR scan config */
    {
        .inquiry_scan_type               = BTM_SCAN_TYPE_STANDARD,                                     /**< Inquiry scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .inquiry_scan_interval           = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                 /**< Inquiry scan interval  (0 to use default) */
        .inquiry_scan_window             = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                   /**< Inquiry scan window (0 to use default) */

        .page_scan_type                  = BTM_SCAN_TYPE_STANDARD,                                     /**< Page scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .page_scan_interval              = WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                    /**< Page scan interval  (0 to use default) */
        .page_scan_window                = WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW                       /**< Page scan window (0 to use default) */
    },

    .ble_scan_cfg =                                                 /* BLE scan settings  */
    {
        .scan_mode                       = BTM_BLE_SCAN_MODE_ACTIVE,                                   /**< BLE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE) */

        /* Advertisement scan configuration */
        .high_duty_scan_interval         = 96,                                                         /**< High duty scan interval */
        .high_duty_scan_window           = 48,                                                         /**< High duty scan window */
        .high_duty_scan_duration         = 30,                                                         /**< High duty scan duration in seconds (0 for infinite) */

        .low_duty_scan_interval          = 2048,                                                       /**< Low duty scan interval  */
        .low_duty_scan_window            = 48,                                                         /**< Low duty scan window */
        .low_duty_scan_duration          = 30,                                                         /**< Low duty scan duration in seconds (0 for infinite) */

        /* Connection scan configuration */
        .high_duty_conn_scan_interval    = 96,                                                         /**< High duty cycle connection scan interval */
        .high_duty_conn_scan_window      = 48,                                                         /**< High duty cycle connection scan window */
        .high_duty_conn_duration         = 30,                                                         /**< High duty cycle connection duration in seconds (0 for infinite) */

        .low_duty_conn_scan_interval     = 2048,                                                       /**< Low duty cycle connection scan interval */
        .low_duty_conn_scan_window       = 48,                                                         /**< Low duty cycle connection scan window */
        .low_duty_conn_duration          = 30,                                                         /**< Low duty cycle connection duration in seconds (0 for infinite) */

        /* Connection configuration */
        .conn_min_interval               = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,                     /**< Minimum connection interval */
        .conn_max_interval               = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,                     /**< Maximum connection interval */
        .conn_latency                    = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                          /**< Connection latency */
        .conn_supervision_timeout        = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,              /**< Connection link supervision timeout */
    },

    .ble_advert_cfg =                                               /* BLE advertisement settings */
    {
        .channel_map                     = BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising channel map (mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39) */
                                           BTM_BLE_ADVERT_CHNL_38 |
                                           BTM_BLE_ADVERT_CHNL_39,

        .high_duty_min_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,            /**< High duty undirected connectable minimum advertising interval */
        .high_duty_max_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,            /**< High duty undirected connectable maximum advertising interval */
        .high_duty_duration              = 30,                                                         /**< High duty undirected connectable advertising duration in seconds (0 for infinite) */

        .low_duty_min_interval           = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,             /**< Low duty undirected connectable minimum advertising interval */
        .low_duty_max_interval           = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MAX_INTERVAL,             /**< Low duty undirected connectable maximum advertising interval */
        .low_duty_duration               = 60,                                                         /**< Low duty undirected connectable advertising duration in seconds (0 for infinite) */

        .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,   /**< High duty directed connectable minimum advertising interval */
        .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,   /**< High duty directed connectable maximum advertising interval */

        .low_duty_directed_min_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,    /**< Low duty directed connectable minimum advertising interval */
        .low_duty_directed_max_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,    /**< Low duty directed connectable maximum advertising interval */
        .low_duty_directed_duration      = 30,                                                         /**< Low duty directed connectable advertising duration in seconds (0 for infinite) */

        .high_duty_nonconn_min_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High duty non-connectable minimum advertising interval */
        .high_duty_nonconn_max_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High duty non-connectable maximum advertising interval */
        .high_duty_nonconn_duration      = 30,                                                         /**< High duty non-connectable advertising duration in seconds (0 for infinite) */

        .low_duty_nonconn_min_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low duty non-connectable minimum advertising interval */
        .low_duty_nonconn_max_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low duty non-connectable maximum advertising interval */
        .low_duty_nonconn_duration       = 0                                                           /**< Low duty non-connectable advertising duration in seconds (0 for infinite) */
    },

    .gatt_cfg =                                                     /* GATT configuration */
    {
        .appearance                     = APPEARANCE_GENERIC_TAG,                                      /**< GATT appearance (see gatt_appearance_e) */
        .client_max_links               = 3,                                                           /**< Client config: maximum number of servers that local client can connect to  */
        .server_max_links               = 3,                                                           /**< Server config: maximum number of remote clients connections allowed by the local */
        .max_attr_len                   = 360,                                                         /**< Maximum attribute length; gki_cfg must have a corresponding buffer pool that can hold this length */
        .max_mtu_size                   = 365                                                          /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
    },

    .rfcomm_cfg =                                                   /* RFCOMM configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of simultaneous connected remote devices*/
        .max_ports                      = 0                                                            /**< Maximum number of simultaneous RFCOMM ports */
    },

    .l2cap_application =                                            /* Application managed l2cap protocol configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */

        /* BR EDR l2cap configuration */
        .max_psm                        = 0,                                                           /**< Maximum number of application-managed BR/EDR PSMs */
        .max_channels                   = 0,                                                           /**< Maximum number of application-managed BR/EDR channels  */

        /* LE L2cap connection-oriented channels configuration */
        .max_le_psm                     = 0,                                                           /**< Maximum number of application-managed LE PSMs */
        .max_le_channels                = 0,                                                           /**< Maximum number of application-managed LE channels */
        /* LE L2cap fixed channel configuration */
        .max_le_l2cap_fixed_channels    = 0                                                            /**< Maximum number of application managed fixed channels supported (in addition to mandatory channels 4, 5 and 6). > */
    },

    .avdt_cfg =
    /* Audio/Video Distribution configuration */
    {
        .max_links                      = 1,                                                           /**< Maximum simultaneous audio/video links */
        .max_seps                       = 3                                                            /**< Maximum number of stream end points */
    },

    .avrc_cfg =                                                     /* Audio/Video Remote Control configuration */
    {
        .roles                          = 1,                                                           /**< Mask of local roles supported (AVRC_CONN_INITIATOR|AVRC_CONN_ACCEPTOR) */
        .max_links                      = 1                                                            /**< Maximum simultaneous remote control links */
    },

    /* LE Address Resolution DB size  */
    .addr_resolution_db_size            = 5,                                                           /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/

    /* Maximum number of buffer pools */
    .max_number_of_buffer_pools         = 6,                                                           /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */

    /* Interval of  random address refreshing */
    .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,            /**< Interval of  random address refreshing - secs */
    /* BLE Filter Accept List size */
    .ble_filter_accept_list_size                = 0,                                                           /**< Maximum number of Filter Accept List devices allowed. Cannot be more than 128 */

};
#endif /* !BTSTACK_VER*/


/*****************************************************************************
 * SDP database for the hci_control application
 ****************************************************************************/
/* SDP Record handle for AVDT Source */
#define HANDLE_AVDT_SOURCE                        0x10001

const uint8_t a2dp_source_sdp_db[A2DP_SOURCE_SDP_DB_SIZE] =
{
    SDP_ATTR_SEQUENCE_1( 63 ),

    /* SDP Record for AVDT Source */
    SDP_ATTR_SEQUENCE_1(61),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVDT_SOURCE),         /* 8 size in bytes */
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_AUDIO_SOURCE),     /*8*/
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),/*3+2*/
            SDP_ATTR_SEQUENCE_1(6),                              /*2*/
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),            /*3*/
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVDTP),              /*3*/
            SDP_ATTR_SEQUENCE_1(6),                              /*2*/
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVDTP),            /*3*/
                SDP_ATTR_VALUE_UINT2(0x103),                     /*3*/
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),/*3+2*/
            SDP_ATTR_SEQUENCE_1(6),                              /*2*/
                SDP_ATTR_UUID16(UUID_SERVCLASS_ADV_AUDIO_DISTRIBUTION),/*3*/
                SDP_ATTR_VALUE_UINT2(0x103),                      /*3*/
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x0001),
};

/*****************************************************************************
 *   codec and audio tuning configurations
 ****************************************************************************/

/* Array of encoder capabilities information. */
wiced_bt_a2dp_codec_info_t bt_audio_codec_capabilities[] =
{
    {
        .codec_id = WICED_BT_A2DP_CODEC_SBC,
        .cie =
        {
            .sbc =
            {
                (A2D_SBC_IE_SAMP_FREQ_44 | A2D_SBC_IE_SAMP_FREQ_48),    /* samp_freq */
                (A2D_SBC_IE_CH_MD_MONO   | A2D_SBC_IE_CH_MD_STEREO |
                 A2D_SBC_IE_CH_MD_JOINT  | A2D_SBC_IE_CH_MD_DUAL),      /* ch_mode */
                (A2D_SBC_IE_BLOCKS_16    | A2D_SBC_IE_BLOCKS_12 |
                 A2D_SBC_IE_BLOCKS_8     | A2D_SBC_IE_BLOCKS_4),        /* block_len */
                (A2D_SBC_IE_SUBBAND_4    | A2D_SBC_IE_SUBBAND_8),       /* num_subbands */
                (A2D_SBC_IE_ALLOC_MD_L   | A2D_SBC_IE_ALLOC_MD_S),      /* alloc_mthd */
                BT_AUDIO_A2DP_SBC_MAX_BITPOOL_44K,                          /* max_bitpool for high quality audio */
                A2D_SBC_IE_MIN_BITPOOL                                  /* min_bitpool */
            }
        }
    },
};

/** A2DP Source configuration data */
wiced_bt_a2dp_source_config_data_t bt_audio_config =
{
#ifdef A2DP_SINK_ENABLE_CONTENT_PROTECTION
    .feature_mask = WICED_BT_A2DP_SINK_FEAT_PROTECT | WICED_BT_A2DP_SOURCE_FEAT_DELAY_RPT,    /* feature mask */
#else
    .feature_mask = WICED_BT_A2DP_SOURCE_FEAT_DELAY_RPT,                                      /* feature mask */
#endif
    .codec_capabilities =
    {
        .count = sizeof(bt_audio_codec_capabilities) / sizeof(bt_audio_codec_capabilities[0]),
        .info  = bt_audio_codec_capabilities,                                           /* codec configuration */
    },
    .default_codec_config = 
	  { .codec_id = WICED_BT_A2DP_CODEC_SBC, .cie =
	      { .sbc =
		{
            A2D_SBC_IE_SAMP_FREQ_44,            /* Sampling Frequency */
            A2D_SBC_IE_CH_MD_JOINT,             /* Channel Mode */
		    A2D_SBC_IE_BLOCKS_16,               /* Block Length */
		    A2D_SBC_IE_SUBBAND_8,               /* Number of Sunbands */
            A2D_SBC_IE_ALLOC_MD_L,              /* Allocation Method */
		    BT_AUDIO_A2DP_SBC_MAX_BITPOOL_44K,  /* Max Bit Pool */
		    A2D_SBC_IE_MIN_BITPOOL              /* Min Bit Pool */
		}
	      }
	},
};

//BD address
wiced_bt_device_address_t bt_device_address = { 0x22, 0x11, 0x44, 0x33, 0x66, 0x55 };

/*
 * wiced_app_cfg_buf_pools_get_num
 */
const wiced_bt_cfg_settings_t *wiced_app_cfg_get_settings(void)
{
    return &a2dp_source_cfg_settings;
}

/*
 * wiced_app_cfg_sdp_record_get
 */
uint8_t *wiced_app_cfg_sdp_record_get(void)
{
    return (uint8_t *)a2dp_source_sdp_db;
}

/*
 * wiced_app_cfg_sdp_record_get_size
 */
uint16_t wiced_app_cfg_sdp_record_get_size(void)
{
    return (uint16_t)sizeof(a2dp_source_sdp_db);
}
