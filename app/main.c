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
 * File Name: main.c
 *
 * Description: Entry file for A2DP Source application.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/
/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "platform_linux.h"
#include "app_bt_utils/app_bt_utils.h"
#include "utils_arg_parser.h"
#include "a2dp_source.h"
#include "log.h"

/******************************************************************************
 *                                MACROS
 *****************************************************************************/
#ifdef TAG
#undef TAG
#endif
#define TAG "[A2DP]"

#define A2DP_SOURCE_NVRAM_ID            WICED_NVRAM_VSID_START

#define WICED_HS_EIR_BUF_MAX_SIZE       (264U)
#define MAX_PATH                        (256U)
#define LOCAL_BDA_LEN                   (50U)
#define IP_ADDR_LEN                     (16U)
#define BT_STACK_HEAP_SIZE              (0xF000U)

#define IP_ADDR                         "000.000.000.000"

#define DISCOVERABLE 					(1u)
#define CONNECTABLE 					(1u)

/******************************************************************************
 *                                EXTERNS
 *****************************************************************************/
extern tAV_APP_CB av_app_cb;
extern wiced_bt_heap_t *p_default_heap;
extern wiced_bt_device_address_t bt_device_address;
extern const wiced_bt_cfg_settings_t a2dp_source_cfg_settings;

/* ***************************************************************************
 *                              GLOBAL VARIABLES
 * **************************************************************************/
static const char app_menu[] = "\n\
================================== \n\
       A2DP Source Menu \n\
----------------------------------\n\
    0.  Exit \n\
    1.  Read Local BD address \n\
    2.  Set Pairing Mode \n\
    3.  Inquiry\n\
    4.  Connect A2DP Sink \n\
    5.  Disconnect A2DP Sink \n\
    6.  Stream Configure \n\
    7.  Stream Start \n\
    8.  Stream Stop \n\
 =================================\n\
Choose option (0-8): \n";
/******************************************************************************
 *                          FUNCTION DEFINITIONS
 ******************************************************************************/

/******************************************************************************
 * Function Name: hci_control_proc_rx_cmd()
 *******************************************************************************
 * Summary:
 *          Function to handle HCI receive
 *
 * Parameters:
 *          uint8_t* p_buffer  : rx buffer
 *          uint32_t length     : rx buffer length
 *
 * Return:
 *          status code
 *
 ******************************************************************************/
uint32_t
hci_control_proc_rx_cmd (uint8_t * p_buffer, uint32_t length)
{
  return 0;
}

/******************************************************************************
 * Function Name: application_start()
 *******************************************************************************
 * Summary:
 *          Application start to initialize stack and other required modules
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 ******************************************************************************/
void
application_start (void)
{
	wiced_result_t wiced_result = WICED_BT_SUCCESS;

#ifdef WICED_BT_TRACE_ENABLE
	/* Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints */
	/* wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE); */

	/* Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart() */
	wiced_set_debug_uart (WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif /* WICED_BT_TRACE_ENABLE */

	/* Register call back and configuration with stack */
	wiced_result =
	wiced_bt_stack_init (a2dp_source_management_callback,
				&a2dp_source_cfg_settings);

	WICED_BT_TRACE ("A2DP Source Start");

	/* Check if stack initialization was successful */

	if (WICED_BT_SUCCESS == wiced_result)
	{
		/* Create a buffer heap, make it the default heap.  */
		p_default_heap = wiced_bt_create_heap ("app", NULL, BT_STACK_HEAP_SIZE,
							NULL, WICED_TRUE);
	}

	if ((WICED_BT_SUCCESS == wiced_result) && (NULL != p_default_heap))
	{
		WICED_BT_TRACE ("Bluetooth Stack Initialization Successful...");
	}
	else				/* Exit App if stack init was not successful or heap creation failed */
	{
		WICED_BT_TRACE ("Bluetooth Stack Initialization or heap creation failed!! Exiting App...");
		exit (EXIT_FAILURE);
	}
}

/******************************************************************************
 * Function Name: APPLICATION_START()
 *******************************************************************************
 * Summary:
 *           BT stack initialization function wrapper
 *
 * Parameters:
 *           None
 *
 * Return:
 *          None
 *
 ******************************************************************************/
void
APPLICATION_START (void)
{
  application_start ();
}

/******************************************************************************
 * Function Name: main()
 *******************************************************************************
 * Summary:
 *          A2DP Source application entry function
 *
 * Parameters:
 *          int argc            : argument count
 *          char *argv[]        : list of arguments
 *
 * Return:
 *          Status code
 *
 ******************************************************************************/
int
main (int argc, char *argv[])
{
	int filename_len = 0;
	char fw_patch_file[MAX_PATH] = {0};
	char hci_port[MAX_PATH] = {0};
	char peer_ip_addr[IP_ADDR_LEN] = "000.000.000.000";
	uint32_t hci_baudrate = 0;
	uint32_t patch_baudrate = 0;
	int btspy_inst = 0;
	uint8_t btspy_is_tcp_socket = 0;	/* Throughput calculation thread handler */
	pthread_t throughput_calc_thread_handle;	/* Audobaud configuration GPIO bank and pin */
	cybt_controller_gpio_config_t autobaud;
	wiced_result_t status = WICED_BT_SUCCESS;
	char ip[10] = {0};
	uint8_t i = 0;
	int in_yes_no;
	uint32_t sample_freq = 0;
	int bda = 0;
	wiced_bt_device_address_t bda_str;

	if (PARSE_ERROR ==
		arg_parser_get_args (argc, argv, hci_port, bt_device_address, &hci_baudrate,
				&btspy_inst, peer_ip_addr, &btspy_is_tcp_socket,
				fw_patch_file, &patch_baudrate, &autobaud))
	{
		return EXIT_FAILURE;
	}

	filename_len = strlen (argv[0]);

	if (filename_len >= MAX_PATH)
	{
		filename_len = MAX_PATH - 1;
	}

	cy_platform_bluetooth_init (fw_patch_file, hci_port, hci_baudrate, patch_baudrate, &autobaud.autobaud_cfg);

	if (fw_patch_file[0])
	{
		WICED_BT_TRACE ("Waiting for downloading patch...");
		wait_controller_reset_ready();
	}

	do
	{
		status = WICED_BT_SUCCESS;
		fprintf (stdout, "%s", app_menu);
		fflush (stdin);
		if (scanf ("%10s", ip) < 0 || strlen(ip) > 1)
		{
			status = WICED_BT_ERROR;
			TRACE_ERR("Input Failed. Status: %d", status);
			continue;
		}
		*ip -= '0';

		switch (*ip)
		{
			case 0:
				/* Exiting application */
				status = WICED_BT_SUCCESS;
				break;

			case 1:
				a2dp_source_bt_print_local_bda ();
				break;

			case 2:
				TRACE_MSG("Allow Pairing (1-Yes, 0-No): ");
				if (scanf ("%d", &in_yes_no) < 0)
				{
					status = WICED_BT_ERROR;
					TRACE_ERR("Input Failed.");
					break;
				}
				if (in_yes_no != 0 && in_yes_no != 1)
				{
					status = WICED_BT_ERROR;
					TRACE_ERR("Input wrong value.");
					break;
				}
				a2dp_source_bt_set_pairability (in_yes_no);
				break;

			case 3:
				TRACE_MSG("Inquiry (1- start, 0- cancel): ");
				if (scanf ("%d", &in_yes_no) < 0)
				{
					status = WICED_BT_ERROR;
					TRACE_ERR("Input Failed.");
					break;
				}
				if (in_yes_no != 0 && in_yes_no != 1)
				{
					status = WICED_BT_ERROR;
					TRACE_ERR("Input wrong value.");
					break;
				}
				status = a2dp_source_bt_inquiry (in_yes_no);
				break;

			case 4:
				TRACE_MSG("Enter the BD Address (XX XX XX XX XX XX): ");
				for (i = 0; i < BD_ADDR_LEN; i++)
				{
					if (scanf ("%x", &bda) < 0)
					{
						status = WICED_BT_ERROR;
						break;
					}
					bda_str[i] = (unsigned char) bda;
				}
				if(status == WICED_BT_ERROR)
				{
					status = WICED_BT_ERROR;
					TRACE_ERR("Input wrong value.");
					break;
				}
				status = a2dp_source_bt_set_visibility (DISCOVERABLE, CONNECTABLE);
				if (status == WICED_BT_SUCCESS)
				{
					status = a2dp_source_command_connect (bda_str, BD_ADDR_LEN);
				}
				break;

			case 5:
				status = a2dp_source_command_disconnect ();
				break;

			case 6:
				TRACE_MSG("Choose Sampling Frequency: (1- 48 kHz, 2- 44.1kHz): ");
				if (scanf ("%u", &sample_freq) < 0)
				{
					status = WICED_BT_ERROR;
					TRACE_ERR("Input Failed.");
					break;
				}
				if ((sample_freq == 1) || (sample_freq == 2))
				{
					a2dp_source_command_stream_config ((uint8_t) sample_freq, AUDIO_CHCFG_JOINT);
				}
				else
				{
					status = WICED_BT_ERROR;
					TRACE_ERR("Input wrong value.");
					break;
				}
				break;

			case 7:
				TRACE_MSG("Starting stream to the existing connection");
				/* When streaming audio to a headset, we will not use 3mbps modulation to improve
				* range.  The 0xcc18 allows all packets types.  Adding 0x2204 tell controller
				* not to use 3DH1, 3DH3 and 3DH5 packets.
				*/
				wiced_bt_dev_setAclPacketTypes (av_app_cb.peer_bda, HCI_PKT_TYPES_MASK_DM5 | HCI_PKT_TYPES_MASK_DH5 |	/* Use 1 mbps 5 slot packets */
								HCI_PKT_TYPES_MASK_DH3 | HCI_PKT_TYPES_MASK_DM3 |	/* Use 1 mbps 3 slot packets */
								HCI_PKT_TYPES_MASK_DH1 | HCI_PKT_TYPES_MASK_DM1 |	/* Use 1 mbps 1 slot packets */
								HCI_PKT_TYPES_MASK_NO_3_DH1 |	/* Don't use 3 mbps 1 slot packets */
								HCI_PKT_TYPES_MASK_NO_3_DH3 |	/* Don't use 3 mbps 3 slot packets */
								HCI_PKT_TYPES_MASK_NO_3_DH5);	/* Don't use 3 mbps 5 slot packets */

				status = a2dp_source_command_stream_start ();
				if (status == WICED_BT_ERROR)
				{
					TRACE_ERR("Stream start failed.");
					break;
				}
				break;

			case 8:
				/* When stopping audio to a headset, we will allow back 3mbps modulation packets
				* 2mbps and 3mbps packets are implicitly enabled (negative logic)
				*/
				wiced_bt_dev_setAclPacketTypes (av_app_cb.peer_bda, HCI_PKT_TYPES_MASK_DM5 | HCI_PKT_TYPES_MASK_DH5 |	/* Use 1 mbps 5 slot packets */
								HCI_PKT_TYPES_MASK_DH3 | HCI_PKT_TYPES_MASK_DM3 |	/* Use 1 mbps 3 slot packets */
								HCI_PKT_TYPES_MASK_DH1 | HCI_PKT_TYPES_MASK_DM1);	/* Use 1 mbps 1 slot packets */
				status = a2dp_source_command_stream_stop ();
				if (status == WICED_BT_ERROR)
				{
					TRACE_ERR("Stream stop failed.");
					break;
				}
				break;

			default:
				status = WICED_BT_ERROR;
				TRACE_ERR("Unknown A2DP Source Command. Choose option from the Menu");
				break;
		}

		if (status == WICED_BT_SUCCESS)
		{
			TRACE_LOG("Command Successful");
		}
		else
		{
			TRACE_ERR("Command Failed. Status: %d", status);
		}

		fflush (stdin);
	} while (*ip != 0);

	TRACE_LOG ("Exiting...\n");
	wiced_bt_delete_heap (p_default_heap);
	wiced_bt_stack_deinit ();

	return EXIT_SUCCESS;
}
