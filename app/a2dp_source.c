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
 * File Name: a2dp_source.c
 *
 * Description: A2DP Source application source file.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif
#include "wiced_bt_a2dp_source.h"
#include "wiced_bt_a2d_sbc.h"
#include "sbc_encoder.h"
#include "sbc_enc_func_declare.h"
#include "sbc_dct.h"
#include "sbc_types.h"
#include "a2dp_source.h"
#include "platform_linux.h"

/******************************************************************************
 *                                MACROS
 ******************************************************************************/
#define A2DP_SOURCE_NVRAM_ID            WICED_NVRAM_VSID_START
#define A2DP_L2CAP_DEFAULT_MTU          (672U)
#define WICED_HS_EIR_BUF_MAX_SIZE       (264U)

#define LINE_SPEED_44K  (328U)
#define LINE_SPEED_48K  (345U)
#define FILE_44K        "44K.wav"
#define FILE_48K        "48K.wav"
#define MAX_KEY_SIZE    (16U)

/* Audio Sample Rate in Hz*/
#define AUDIO_SAMPLE_RATE_48kHz            (48000U)
#define AUDIO_SAMPLE_RATE_44_1kHz          (44100U)
#define AUDIO_SAMPLE_RATE_32kHz            (32000U)
#define AUDIO_SAMPLE_RATE_16kHz            (16000U)

#define SEC1000_MICROSEC                (1000000000U)
#define FRAME_TIME_STAMP                (90000U)
#define MARKER_PAYLOAD_BYTE             (0x60U)
#define INQUIRY_DURATION                (5U)
#define BUFF_LEN_4                      (4U)

#define BUFF_SIZE_TO_READ_CHUNK     (1024U)

#define LEN_RIFF                (12U)
#define SHIFT_FILE_SIZE         (4U)
#define LEN_CHUNK_ID            (4U)
#define LEN_CHUNK_SIZE          (4U)
#define SHIFT_NUM_CHANNELS      (2U)
#define SHIFT_SAMPLE_RATE       (4U)
#define SHIFT_BLOCK_ALIGN       (12U)
#define SHIFT_BYTE_PER_SAMPLE   (14U)
/******************************************************************************
 *                          STRUCTURES AND ENUMERATIONS
 ******************************************************************************/
typedef enum
{
    FMT_RIFF = 0,
    FMT_FILESIZE,
    FMT_WAVE,
    FMT_MRK,
    FMT_LENFMT,
    FMT_TYPE,
    FMT_NUMCH,
    FMT_SAMPLERATE,
    FMT_BYTERATE,
    FMT_BLKALIGN,
    FMT_BYTESPERSAMPLE,
    FMT_DATASIZE
}WAV_FORMAT_HDR;

/******************************************************************************
 *                                GLOBAL VARIABLES
 ******************************************************************************/
static SBC_ENC_PARAMS   strEncParams = { 0 };
static uint16_t handle = 0x0000; /* A2DP Connection Handle */
uint8_t outBuf[A2DP_L2CAP_DEFAULT_MTU]; /*L2cap default MTU size*/
static int  SampleRate      = AUDIO_SAMPLE_RATE_44_1kHz;
static uint64_t  usTimeout;
static uint8_t   *pPCM = NULL;
static uint8_t   *pCurPCM = NULL;
static uint32_t  PCMDataSize = 0;
static uint32_t dwPCMBytesPerFrame;
static uint32_t Timestamp;
static uint32_t TimestampInc;
static uint8_t  stream_avdt_handle;
static uint32_t frame_per_packet;
tAV_APP_CB      av_app_cb;
sem_t stream_buff_sem;
uint16_t delay_reported_from_sink_micro_sec = 0;
unsigned int    frame_size;
pthread_t a2dp_src_thread_id;
volatile wiced_bool_t bStreamingActive = WICED_FALSE;
wiced_bt_heap_t *p_default_heap = NULL;
uint8_t pincode[WICED_PIN_CODE_LEN] = { 0x30, 0x30, 0x30, 0x30 };

/******************************************************************************
 *                                EXTERNS
 ******************************************************************************/
extern uint32_t getTick();
extern const uint8_t                        a2dp_source_sdp_db[A2DP_SOURCE_SDP_DB_SIZE];
extern const wiced_bt_cfg_settings_t        a2dp_source_cfg_settings;
extern wiced_bt_a2dp_source_config_data_t   bt_audio_config;
extern wiced_bt_device_address_t            bt_device_address;
/******************************************************************************
 *                          FUNCTION DECLARATIONS
 ******************************************************************************/
void* a2dp_source_audio_thread (void* arg);
static uint8_t a2dp_source_encode_sendSBC ();
static void a2dp_source_parse_sbc_params (wiced_bt_a2d_sbc_cie_t * pSbc, uint32_t *pSf,
                                          uint32_t *pBlocks, uint32_t *pSubbands, uint32_t *pMode);
uint16_t a2dp_source_calculate_delay ();

/******************************************************************************
 *                          FUNCTION DEFINITIONS
 ******************************************************************************/

/* ****************************************************************************
 * Function Name: get_state_name
 ******************************************************************************
 * Summary:
 *          Gives the string equivalent for the State
 *
 * Parameters:
 *          state - AV State
 *
 * Return:
 *          Pointer to String containing the State Name
 *
 * ***************************************************************************/
static const char *get_state_name(AV_STATE state)
{
    switch((int)state)
    {
        CASE_RETURN_STR(AV_STATE_IDLE)
        CASE_RETURN_STR(AV_STATE_CONFIGURE)
        CASE_RETURN_STR(AV_STATE_OPEN)
        CASE_RETURN_STR(AV_STATE_STARTED)
        CASE_RETURN_STR(AV_STATE_RECONFIG)
        CASE_RETURN_STR(AV_STATE_DISCONNECTING)
    }

    return NULL;
}

/* ****************************************************************************
 * Function Name: get_audio_freq_name
 *******************************************************************************
 * Summary:
 *          Gives the string equivalent for the Event type
 *
 * Parameters:
 *          freq - Sample Frequency type
 *
 * Return:
 *          Pointer to String containing the Frequency Name
 *
 * ***************************************************************************/
static const char *get_audio_freq_name(uint8_t freq)
{
    switch((int)freq)
    {
        CASE_RETURN_STR(A2D_SBC_IE_SAMP_FREQ_16)
        CASE_RETURN_STR(A2D_SBC_IE_SAMP_FREQ_32)
        CASE_RETURN_STR(A2D_SBC_IE_SAMP_FREQ_44)
        CASE_RETURN_STR(A2D_SBC_IE_SAMP_FREQ_48)
    }

    return NULL;
}

/* ****************************************************************************
 * Function Name: get_audio_chcfg_name
 *******************************************************************************
 * Summary:
 *          Gives the string equivalent for the Event type
 *
 * Parameters:
 *          chcfg - Channel Configuration type
 *
 * Return:
 *          Pointer to String containing the Channel Configuration Name
 *
 * ***************************************************************************/
static const char *get_audio_chcfg_name(uint8_t chcfg)
{
    switch((int)chcfg)
    {
        CASE_RETURN_STR(A2D_SBC_IE_CH_MD_MONO)
        CASE_RETURN_STR(A2D_SBC_IE_CH_MD_DUAL)
        CASE_RETURN_STR(A2D_SBC_IE_CH_MD_STEREO)
        CASE_RETURN_STR(A2D_SBC_IE_CH_MD_JOINT)
    }

    return NULL;
}

/* ****************************************************************************
 * Function Name: a2dp_source_deinit_streaming
 ******************************************************************************
 * Summary:
 *        De-initialize the buffers and variables used for Audio Streaming state
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_deinit_streaming()
{
    WICED_BT_TRACE ("De-iniatializing the stream buffer \n");
    bStreamingActive = FALSE;
    pCurPCM = NULL;
    free (pPCM);
    pPCM = NULL;
}

/* ****************************************************************************
 * Function Name: a2dp_source_update_sink_rep_delay
 ******************************************************************************
 * Summary:
 *          Updates the delay report value to the latest as per the Sink
 *
 * Parameters:
 *          delay_ms: Delay in Milliseconds
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_update_sink_rep_delay(uint16_t delay_ms)
{
    WICED_BT_TRACE("Updated Sink reported delay %u ms", delay_ms);
    delay_reported_from_sink_micro_sec += delay_ms * 1000;
}

/* ****************************************************************************
 * Function Name: file_reader
 ******************************************************************************
 * Summary:
 *          To read file and check if it works without error.
 *
 * Parameters:
 *          buff: buffer to store data
 *          ezise: elem size
 *          num: number of elems
 *          ptr: pointer to the file
 *
 * Return:
 *          rtn: num of data
 *
 * ***************************************************************************/
size_t file_reader(void *buff, uint8_t esize, uint32_t num, FILE *ptr)
{
    size_t rtn = fread((uint8_t*)buff, esize, num, ptr);
    if (rtn != num)
    {
        WICED_BT_TRACE("Error: while reading wave file\n");
        return 0;
    }
    return rtn;
}

/* ****************************************************************************
 * Function Name: a2dp_source_load_wav_file
 ******************************************************************************
 * Summary:
 *          Helper function to read the file contents of Audio Source file
 *
 * Parameters:
 *          ptr: Pointer to opened file to read from
 *          num_bytes: Number of bytes to be read
 *
 * Return:
 *          val: Value of the file content at a specific position based on index
 *
 * ***************************************************************************/
uint8_t a2dp_source_load_wav_file(char *file_name)
{
    FILE *ptr = fopen(file_name, "rb");
    int format_type;
    int num_channels;
    int sample_rate;
    int block_align;
    int bytes_per_sample;
    int total_file_size;
    uint32_t read_size = 0;
    uint32_t data_size = 0;
    uint32_t chunk_size = 0;
    WAV_FORMAT_HDR fmt_hdr = FMT_RIFF;
    uint8_t buff[BUFF_SIZE_TO_READ_CHUNK] = {0};

    if (ptr == NULL)
    {
        WICED_BT_TRACE("Error: while opening %s wave file\n", file_name);
        return A2DP_APP_FAILED;
    }

    // RIFF
    file_reader(buff, 1, LEN_RIFF, ptr);
    total_file_size = *(uint32_t*)(buff + SHIFT_FILE_SIZE);
    WICED_BT_TRACE("Total file size: %d \n", total_file_size);

    // fmt
    file_reader(buff, 1, LEN_CHUNK_ID, ptr);
    file_reader(&chunk_size, 1, LEN_CHUNK_SIZE, ptr);

    memset(buff, 0, 12);
    if (chunk_size < BUFF_SIZE_TO_READ_CHUNK)
    {
        file_reader(buff, 1, chunk_size, ptr);
    }
    else
    {
        WICED_BT_TRACE("Error: while file reading.\n");
        fclose (ptr);
        return A2DP_APP_FAILED;
    }

    format_type         = *(uint16_t*) buff;
    num_channels        = *(uint16_t*)(buff + SHIFT_NUM_CHANNELS);
    sample_rate         = *(uint32_t*)(buff + SHIFT_SAMPLE_RATE);
    block_align         = *(uint16_t*)(buff + SHIFT_BLOCK_ALIGN);
    bytes_per_sample    = *(uint16_t*)(buff + SHIFT_BYTE_PER_SAMPLE) >> 3;

    if (format_type != 1)
    {
        WICED_BT_TRACE("Error: invalid format type %d\n", format_type);
        fclose (ptr);
        return A2DP_APP_FAILED;
    }
    if (num_channels != 2)
    {
        WICED_BT_TRACE("Error: number of channels check failed %d\n", num_channels);
        fclose (ptr);
        return A2DP_APP_FAILED;
    }

    // sub chunks
    for (;;)
    {
        file_reader(buff, 1, 4, ptr);
        file_reader(&chunk_size, 1, 4, ptr);
        if (buff[0] == 'd' && buff[1] == 'a' && buff[2] == 't' && buff[3] == 'a')
        {
            data_size = chunk_size;
            break;
        }
        if (0 != fseek(ptr, chunk_size, SEEK_CUR))
        {
            printf("Error: fseek failed for format header %d\n", fmt_hdr);
            fclose (ptr);
            return A2DP_APP_FAILED;
        }
    }

    if (data_size < (1 << 31) && data_size > 0)
    {
        pPCM = (uint8_t *)malloc (data_size);
    }
    else
    {
        WICED_BT_TRACE ("Error: data size out of range\n");
    }
    if (pPCM == NULL)
    {
        WICED_BT_TRACE ("Error: memory allocation for PCM Data failed \n");
    }

    if (data_size < (1 << 31) && data_size > 0)
    {
        PCMDataSize = fread (pPCM, 1, data_size, ptr);
    }
    else
    {
        WICED_BT_TRACE ("Error: data size out of range\n");
    }
    fclose (ptr);

    if (PCMDataSize != 0)
    {
        return A2DP_APP_SUCCESS;
    }
    else
    {
        return A2DP_APP_FAILED;
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_start_streaming
 ******************************************************************************
 * Summary:
 *          Loads the required .wav file to memory, initializes the Enocder,
 *          Starts a thread for encoding and streaming audio
 *
 * Parameters:
 *          bActivate: State of Streaming as requested by the user
 *          avdt_handle: AVDTP Connection handle as received in the event
 *                       WICED_BT_A2DP_SOURCE_START_IND_EVT
 *          p_stream_cfg: CODEC Configuration
 *          peer_mtu: Stream MTU of the Peer side
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_start_streaming (wiced_bool_t bActivate, uint8_t avdt_handle,
                                  wiced_bt_a2d_sbc_cie_t *p_stream_cfg, uint16_t peer_mtu)
{
    static int  NumBlocks       = SBC_BLOCK_3;
    static int  NumSubBands     = SUB_BANDS_8;
    static int  NumChannels     = SBC_MAX_NUM_OF_CHANNELS;
    static int  Mode            = SBC_JOINT_STEREO;

    uint8_t file_read_done;
    unsigned int fr_per_1000sec, pkts_per_1000sec;
    int join;
    int thread_rtn = 0;

    /* Safety check in case we are called more than once to do the same thing */
    if (bActivate == bStreamingActive)
    {
        WICED_BT_TRACE ("Streaming State: Already in %s",
                      bActivate ? "ACTIVE" : "SUSPEND");
        return;
    }

    WICED_BT_TRACE ("Streaming State: setting to %s \n",
                  bActivate ? "ACTIVE" : "SUSPEND");

    if (!bActivate)
    {
        bStreamingActive = FALSE;
        return;
    }

    stream_avdt_handle = avdt_handle;

    WICED_BT_TRACE ("Streaming parameters: %u  samplesPerSec: %u  peer_mtu: %u",
                  bActivate, p_stream_cfg->samp_freq, peer_mtu);

    if (pPCM == NULL)
    {
        if (p_stream_cfg->samp_freq == A2D_SBC_IE_SAMP_FREQ_48)
        {
            file_read_done = a2dp_source_load_wav_file (FILE_48K);
        }
        else
        {
            file_read_done = a2dp_source_load_wav_file (FILE_44K);
        }

        if (A2DP_APP_FAILED == file_read_done)
        {
            WICED_BT_TRACE ("ERROR - Unable to open: %s!",
                          p_stream_cfg->samp_freq == A2D_SBC_IE_SAMP_FREQ_48 ?
                                  FILE_48K : FILE_44K);
            return;
        }

        WICED_BT_TRACE (
                "Read %u bytes from: %s",
                PCMDataSize,
                p_stream_cfg->samp_freq == A2D_SBC_IE_SAMP_FREQ_48 ?
                        "48K.pcm" : "44K.pcm");
    }

    /* Extract the codec parameters */
    a2dp_source_parse_sbc_params (p_stream_cfg, &SampleRate, &NumBlocks, &NumSubBands, &Mode);

    WICED_BT_TRACE ("SampleRate: %u  NumBlocks: %u  NumSubBands: %u  Mode: %u",
                    SampleRate, NumBlocks, NumSubBands, Mode);

    NumChannels = Mode >= SBC_STEREO ? SBC_STEREO : SBC_DUAL;
    join = Mode == SBC_JOINT_STEREO ? SBC_DUAL : SBC_MONO;

    if (SampleRate <= AUDIO_SAMPLE_RATE_44_1kHz)
    {
        strEncParams.s16SamplingFreq = SBC_sf44100;
    }
    else
    {
        strEncParams.s16SamplingFreq = SBC_sf48000;
    }

    strEncParams.s16AllocationMethod =
    p_stream_cfg->alloc_mthd == A2D_SBC_IE_ALLOC_MD_S ? SBC_SNR : SBC_LOUDNESS;
    strEncParams.s16ChannelMode = Mode;
    strEncParams.s16NumOfBlocks = NumBlocks;
    strEncParams.s16NumOfSubBands = NumSubBands;

    // Have to check decoder
    if (SampleRate == AUDIO_SAMPLE_RATE_44_1kHz)
    {
        strEncParams.u16BitRate = LINE_SPEED_44K;
    }
    if (SampleRate == AUDIO_SAMPLE_RATE_48kHz)
    {
        strEncParams.u16BitRate = LINE_SPEED_48K;
    }

    SBC_Encoder_Init (&strEncParams);

    /*FORMULA as in A2DP Spec:
     *
     * frame_length = 4 + (4 * nrof_subbands * nrof_channels ) / 8
     * nrof_blocks * nrof_channels * bitpool / 8 .
     * for the MONO and DUAL_CHANNEL channel modes, and
     * frame_length = 4 + (4 * nrof_subbands * nrof_channels ) / 8
     * (join * nrof_subbands + nrof_blocks * bitpool ) / 8
     */

    /* Calculate the periodicity to compress and send data in microseconds */
    frame_size = 4 + ((4 * NumSubBands * 2) >> 3)
    + ((join * NumSubBands + NumBlocks * strEncParams.s16BitPool) >> 3)
    + !!((join * NumSubBands + NumBlocks * strEncParams.s16BitPool) & 7);

    frame_per_packet = (peer_mtu - AVDT_MEDIA_OFFSET) / frame_size;

    if (frame_per_packet > A2D_SBC_HDR_NUM_MSK)
      frame_per_packet = A2D_SBC_HDR_NUM_MSK;

    if (frame_per_packet == 0)
      frame_per_packet = 1;

    fr_per_1000sec = (SampleRate * 1000) / (NumSubBands * NumBlocks);
    pkts_per_1000sec = fr_per_1000sec / frame_per_packet;
    usTimeout = SEC1000_MICROSEC / pkts_per_1000sec;
    usTimeout = usTimeout * 100 / 108;

    /*NumSubBands * NumBlocks * NumChannels* (bits_per_sample/2 )*/
    dwPCMBytesPerFrame = 2 * NumSubBands * NumBlocks * NumChannels;

    WICED_BT_TRACE ("Stream Starting:  pcm_bytes_per_frame %u \n", dwPCMBytesPerFrame);

    Timestamp = 0;
    TimestampInc = ((NumBlocks * NumSubBands * FRAME_TIME_STAMP) / SampleRate) * frame_per_packet;

    WICED_BT_TRACE ("Stream Parameters: %d  Blk: %d   SB: %d  BitP: %d  uS: %d  Size: %d  Per: %d  fp1000s: %u \n",
                    SampleRate, NumBlocks, NumSubBands, strEncParams.s16BitPool, usTimeout,
                    frame_size, frame_per_packet, fr_per_1000sec);

    if (!pCurPCM)
      pCurPCM = pPCM;

    bStreamingActive = TRUE;

    if (0 != sem_init(&stream_buff_sem, 0, 0))
    {
         WICED_BT_TRACE("Semaphore init has failed\n");
    }
    thread_rtn = pthread_create (&a2dp_src_thread_id, NULL, a2dp_source_audio_thread, NULL);
    if(thread_rtn != 0)
    {
        WICED_BT_TRACE("Thread create failed");
    }
    if (0 != sem_post(&stream_buff_sem))
    {
        WICED_BT_TRACE("Semaphore post failed\n");
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_encode_sendSBC
 ******************************************************************************
 * Summary:
 *          Calls Encoder to encode the PCM data and sends to Stack
 *
 * Parameters:
 *          None
 * Return:
 *          A2DP_APP_SUCCESS if the data is sent successfully
 *          A2DP_APP_FAILED if there was an error
 *
 * ***************************************************************************/
uint8_t a2dp_source_encode_sendSBC ()
{
    uint16_t    result = A2DP_APP_SUCCESS;
    uint8_t     *pSBC;
    uint32_t    i;
    uint16_t    pkt_len = 0;

    /* Race condition on stopping streaming */
    if (pPCM == NULL)
    {
        WICED_BT_TRACE("PCM Data Buffer is NULL\n");
        return A2DP_APP_FAILED;
    }

    if (0 != sem_wait(&stream_buff_sem))
    {
        WICED_BT_TRACE("Semaphore wait failed\n");
        return A2DP_APP_FAILED;
    }

    pkt_len = frame_size * frame_per_packet;

    pSBC = outBuf;
    *pSBC++ = frame_per_packet;
    for (i = 0; i < frame_per_packet; i++)
    {
        strEncParams.as16PcmBuffer = (SINT16 *)pCurPCM;
        strEncParams.pu8Packet     = pSBC;
        SBC_Encoder (&strEncParams);
        pSBC += strEncParams.u16PacketLength;

        pCurPCM += dwPCMBytesPerFrame;

        /* Check if the Current buffer read position is crossing the datasize */
        if (((pCurPCM - pPCM) + dwPCMBytesPerFrame) > PCMDataSize)
        {
            /* Reset to beginning of the PCM data */
            pCurPCM = pPCM;
        }
    }

    linux_stack_lock(NULL);
    result = wiced_bt_avdt_write_req (stream_avdt_handle, outBuf, (uint16_t)(pSBC - outBuf),
                                      Timestamp, MARKER_PAYLOAD_BYTE, 0);
    linux_stack_unlock(NULL);

    if (result != AVDT_SUCCESS)
    {
        result = A2DP_APP_FAILED;
    }

    Timestamp += TimestampInc;
    return result;
}

/* ****************************************************************************
 * Function Name: a2dp_source_calculate_delay
 ******************************************************************************
 * Summary:
 *          Calculate the required delay to wait for next audio write
 *
 * Parameters:
 *          None
 * Return:
 *          delay_in_micro_sec: Delay in microseconds
 *
 * ***************************************************************************/
uint16_t a2dp_source_calculate_delay()
{
    float bytes_per_pkt = 0;
    float delay_in_micro_sec = 0;

    bytes_per_pkt = frame_size * frame_per_packet;
    delay_in_micro_sec = ((bytes_per_pkt * 1000) / (SampleRate *2)) * 1000;
    WICED_BT_TRACE (" Required delay is %d between pkts of size %d to achieve SampleRate %d",
                    (uint16_t)delay_in_micro_sec, (uint16_t)bytes_per_pkt, SampleRate);
    return (uint16_t)delay_in_micro_sec;
}

/* ****************************************************************************
 * Function Name: a2dp_source_audio_thread
 ******************************************************************************
 * Summary:
 *          Thread to handle the audio encoding and streaming
 *
 * Parameters:
 *          NULL
 * Return:
 *          NULL
 *
 * ***************************************************************************/
void* a2dp_source_audio_thread(void* arg)
{
    WICED_BT_TRACE("a2dp_src_thread usTimeout: %u \n", usTimeout);

    /* Initial Burst */
    if (A2DP_APP_FAILED == a2dp_source_encode_sendSBC())
    {
        WICED_BT_TRACE("Encode and Send failed. \n");
    }

    while (bStreamingActive)
    {
        if (!bStreamingActive)
            break;

        if (A2DP_APP_FAILED == a2dp_source_encode_sendSBC())
        {
            WICED_BT_TRACE("Encode and Send failed. \n");
        }
        usleep(usTimeout);
    }

    WICED_BT_TRACE("a2dp_src_thread() exit");
    return NULL;
}

/* ****************************************************************************
 * Function Name: a2dp_source_parse_sbc_params
 ******************************************************************************
 * Summary:
 *          Parse SBC Parameters and set teh configuration for Encoder
 *
 * Parameters:
 *          NULL
 * Return:
 *          NULL
 *
 * ***************************************************************************/
static void a2dp_source_parse_sbc_params (wiced_bt_a2d_sbc_cie_t * pSbc, uint32_t *pSf,
                                          uint32_t *pBlocks, uint32_t *pSubbands, uint32_t *pMode)
{
    switch (pSbc->samp_freq)
    {
    case A2D_SBC_IE_SAMP_FREQ_16:
        *pSf = AUDIO_SAMPLE_RATE_16kHz;
        break;
    case A2D_SBC_IE_SAMP_FREQ_32:
        *pSf = AUDIO_SAMPLE_RATE_32kHz;
        break;
    case A2D_SBC_IE_SAMP_FREQ_44:
        *pSf = AUDIO_SAMPLE_RATE_44_1kHz;
        break;
    case A2D_SBC_IE_SAMP_FREQ_48:
        *pSf = AUDIO_SAMPLE_RATE_48kHz;
        break;
    }

    switch (pSbc->block_len)
    {
    case A2D_SBC_IE_BLOCKS_4:
        *pBlocks = SBC_BLOCK_0;
        break;
    case A2D_SBC_IE_BLOCKS_8:
        *pBlocks = SBC_BLOCK_1;
        break;
    case A2D_SBC_IE_BLOCKS_12:
        *pBlocks = SBC_BLOCK_2;
        break;
    case A2D_SBC_IE_BLOCKS_16:
        *pBlocks = SBC_BLOCK_3;
        break;
    }

    switch (pSbc->num_subbands)
    {
    case A2D_SBC_IE_SUBBAND_4:
        *pSubbands = SUB_BANDS_4;
        break;
    case A2D_SBC_IE_SUBBAND_8:
        *pSubbands = SUB_BANDS_8;
        break;
    }

    switch (pSbc->ch_mode)
    {
    case A2D_SBC_IE_CH_MD_MONO:
        *pMode = SBC_MONO;
        break;
    case A2D_SBC_IE_CH_MD_DUAL:
        *pMode = SBC_DUAL;
        break;
    case A2D_SBC_IE_CH_MD_STEREO:
        *pMode = SBC_STEREO;
        break;
    case A2D_SBC_IE_CH_MD_JOINT:
        *pMode = SBC_JOINT_STEREO;
        break;
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_set_audio_streaming
 ******************************************************************************
 * Summary:
 *          Set Audio Streaming state and initiate the action accordingly.
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
static void a2dp_source_set_audio_streaming(uint8_t handle, wiced_bool_t start_audio)
{
    WICED_BT_TRACE("Setting Audio Stream State to %d \n", start_audio);

    if (start_audio)
    {
        wiced_bt_dev_cancel_sniff_mode(av_app_cb.peer_bda);
        a2dp_source_start_streaming(WICED_TRUE, handle, &av_app_cb.codec_config.cie.sbc, av_app_cb.stream_mtu);
    }
    else
    {
        a2dp_source_start_streaming(WICED_FALSE, handle, NULL, 0);
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_control_cback
 ******************************************************************************
 * Summary:
 *          Control callback supplied by  the a2dp source profile code.
 *
 * Parameters:
 *          event - control event called back
 *          p_data - event data
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void  a2dp_source_control_cback (wiced_bt_a2dp_source_event_t event, wiced_bt_a2dp_source_event_data_t *p_data)
{
    switch (event)
    {
    case WICED_BT_A2DP_SOURCE_CONNECT_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        /**< Connected event, received on establishing connection to a peer device. Ready to stream. */
        if (p_data->connect.result == WICED_SUCCESS)
        {
            /* Save the address of the remote device on remote connection */
            memcpy (av_app_cb.peer_bda, p_data->connect.bd_addr,
                sizeof(wiced_bt_device_address_t));
            av_app_cb.lcid = p_data->connect.lcid;

            /* Maintain State */
            av_app_cb.state = AV_STATE_OPEN;

            WICED_BT_TRACE ("Connected to addr: <%B> Handle %d \n\r",p_data->connect.bd_addr,
                            p_data->connect.handle);

            handle = p_data->connect.handle;
        }
        else
        {
            WICED_BT_TRACE (" a2dp source connection to <%B> failed %d \n", p_data->connect.bd_addr,
                  p_data->connect.result);
        }
        break;

    case WICED_BT_A2DP_SOURCE_DISCONNECT_EVT:
        /**< Disconnected event, received on disconnection from a peer device */
        /* Maintain State */
        av_app_cb.state = AV_STATE_IDLE;
        a2dp_source_deinit_streaming();
        handle = 0; /* reset connection handle */
        WICED_BT_TRACE (" a2dp source disconnected \n");
        break;

    case WICED_BT_A2DP_SOURCE_CONFIGURE_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        /* Maintain State */
        av_app_cb.state = AV_STATE_CONFIGURE;
        memcpy (&av_app_cb.codec_config, p_data->set_config.codec_config,
          sizeof(wiced_bt_a2dp_codec_info_t));
        av_app_cb.stream_mtu = p_data->set_config.stream_mtu;
        break;

    case WICED_BT_A2DP_SOURCE_START_IND_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        if (!wiced_bt_a2dp_source_send_start_response (p_data->start_ind.handle,
                             p_data->start_ind.label,
                             A2D_SUCCESS))
        {
            if (av_app_cb.state != AV_STATE_STARTED)
            {
                WICED_BT_TRACE (" a2dp source streaming started handle:%d lcid %x \n",
                  p_data->start_cfm.handle, av_app_cb.lcid);
                a2dp_source_set_audio_streaming (p_data->start_ind.handle, WICED_TRUE);
                av_app_cb.state = AV_STATE_STARTED;
            }
            else
            {
                WICED_BT_TRACE (" a2dp source streaming already started \n");
            }
        }
        break;

    case WICED_BT_A2DP_SOURCE_START_CFM_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        /*Start stream event, received when audio streaming is about to start*/
        if (p_data->start_cfm.result != WICED_SUCCESS)
        {
            WICED_BT_TRACE ("Stream start Error \n");
            break;
        }
        if (av_app_cb.state != AV_STATE_STARTED)
        {
            WICED_BT_TRACE (" a2dp source streaming started handle:%d lcid %x \n",
              p_data->start_cfm.handle, av_app_cb.lcid);
            a2dp_source_set_audio_streaming (p_data->start_cfm.handle, WICED_TRUE);
            av_app_cb.state = AV_STATE_STARTED;
        }
        else
        {
            WICED_BT_TRACE (" a2dp source streaming already started \n");
        }
        break;

    case WICED_BT_A2DP_SOURCE_SUSPEND_EVT:
        /**< Suspend stream event, received when audio streaming is suspended */
        /* Maintain State */
        av_app_cb.state = AV_STATE_OPEN;
        WICED_BT_TRACE (" a2dp source streaming suspended \n");
        a2dp_source_set_audio_streaming (p_data->suspend.handle, WICED_FALSE);
        break;

    case WICED_BT_A2DP_SOURCE_WRITE_CFM_EVT:
        WICED_BT_TRACE (" a2dp source write cfm event \n");
        if (0 !=sem_post(&stream_buff_sem))
        {
            WICED_BT_TRACE("Semaphore post failed\n");
        }
        break;

    case WICED_BT_A2DP_SOURCE_DELAY_RPT_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        WICED_BT_TRACE ("a2dp source WICED_BT_A2DP_SOURCE_DELAY_RPT_EVT delay is %d\n",
              p_data->delay_ms);
        a2dp_source_update_sink_rep_delay (p_data->delay_ms);
        break;

    default:
        WICED_BT_TRACE ("a2dp source Unhandled Event %d\n", event);
        break;
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_inquiry_result_cback
 ******************************************************************************
 * Summary:
 *          Handle Inquiry result callback from teh stack, format and
 *          send event over UART
 *
 * Parameters:
 *          p_inquiry_result - Inquiry result consisting BD address,
 *                             RSSI and Device class
 *          p_eir_data - EIR data
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_bt_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data )
{
    int       i;
    uint8_t   len; /* length of EIR data */
    uint16_t  code;

    if (p_inquiry_result == NULL)
    {
        WICED_BT_TRACE( "Inquiry complete \n");
    }
    else
    {
        WICED_BT_TRACE( "Inquiry result: ");
        print_bd_address (p_inquiry_result->remote_bd_addr );
        WICED_BT_TRACE_ARRAY((uint8_t*)(p_inquiry_result->dev_class), 3, "COD :" );
        WICED_BT_TRACE( "RSSI: ", p_inquiry_result->rssi );

        /* currently callback does not pass the data of the adv data,
         * need to go through the data
         * zero len in the LTV means that there is no more data
         */
        if (( p_eir_data != NULL ) && ( len = *p_eir_data ) != 0)
        {
            WICED_BT_TRACE_ARRAY( ( uint8_t* )( p_eir_data ), len, "EIR :" );
        }
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_inquiry
 ******************************************************************************
 * Summary:
 *        Handle Inquiry command from user
 *
 * Parameters:
 *        enable - Enable Inquiry if 1, Cancel if 0
 *
 * Return:
 *        wiced_result_t: result of start or cancel inquiry operation initiation
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_bt_inquiry( uint8_t enable )
{
    wiced_result_t           result;
    wiced_bt_dev_inq_parms_t params;

    if ( enable )
    {
        memset( &params, 0, sizeof( params ) );

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = INQUIRY_DURATION;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry( &params, &a2dp_source_bt_inquiry_result_cback );
        if (result == WICED_BT_PENDING)
        {
            result = WICED_BT_SUCCESS;
        }
        WICED_BT_TRACE( "Inquiry started:%d\n", result );
    }
    else
    {
        result = wiced_bt_cancel_inquiry( );
        WICED_BT_TRACE( "Cancel inquiry:%d\n", result );
    }
    return result;
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_set_visibility
 ******************************************************************************
 * Summary:
 *          Handle Set Visibility command
 *
 * Parameters:
 *          discoverability: Discoverable if 1, Non-discoverable if 0
 *          connectability: Connectable if 1, Non-connectable if 0
 *
 * Return:
 *          wiced_result_t: result of set visibility operation
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_bt_set_visibility( uint8_t discoverability, uint8_t connectability )
{
    wiced_result_t status = WICED_BT_SUCCESS;

    if ((discoverability > 1) || (connectability > 1))
    {
        WICED_BT_TRACE( "Invalid Input \n");
        status = WICED_BT_ERROR;
    }
    else if ((discoverability != 0) && (connectability == 0))
    {
        /* we cannot be discoverable and not connectable */
        WICED_BT_TRACE("we cannot be discoverable and not connectable \n");
        status = WICED_BT_ERROR;
    }
    else
    {
        wiced_bt_dev_set_discoverability((discoverability != 0) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE,
                                            BTM_DEFAULT_DISC_WINDOW,
                                            BTM_DEFAULT_DISC_INTERVAL);

        wiced_bt_dev_set_connectability((connectability != 0) ? WICED_TRUE : WICED_FALSE,
                                            BTM_DEFAULT_CONN_WINDOW,
                                            BTM_DEFAULT_CONN_INTERVAL);
    }
    return status;
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_set_pairability
 ******************************************************************************
 * Summary:
 *          Handle Set Pairability command
 *
 * Parameters:
 *          pairing_allowed: Pairing allowed if 1, not allowed if 0
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_bt_set_pairability( uint8_t pairing_allowed )
{
    wiced_bt_set_pairable_mode(pairing_allowed, TRUE);
        WICED_BT_TRACE( "Set the pairing allowed to %d \n", pairing_allowed );
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_print_local_bda
 ******************************************************************************
 * Summary:
 *          Print Local BDA command
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_bt_print_local_bda( void )
{
    wiced_bt_device_address_t bda = { 0 };

    wiced_bt_dev_read_local_addr(bda);
    WICED_BT_TRACE( "Local Bluetooth Device Address:");
    print_bd_address (bda);
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_connect
 ******************************************************************************
 * Summary:
 *          Connects to the A2DP Sink with the given BD-Address
 *
 * Parameters:
 *          bd_addr: Remote BD Address
 *          len: Length of the BD-address.
 *
 * Return:
 *          status: result of a2dp connect API
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_command_connect(wiced_bt_device_address_t bd_addr, uint32_t len)
{
    wiced_result_t status = WICED_BT_SUCCESS;

    if (handle > 0)
    {
        WICED_BT_TRACE("Already Connected \n\r");
        status = WICED_BT_ERROR;
    }
    else
    {
        WICED_BT_TRACE("Connecting to [%B] \n\r", bd_addr );
        status = wiced_bt_a2dp_source_connect(bd_addr);
    }
    return status;
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_disconnect
 ******************************************************************************
 * Summary:
 *          Disconnects the A2DP Sink that is already connected
 *
 * Parameters:
 *          None
 *
 * Return:
 *          status: result of a2dp disconnect API
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_command_disconnect()
{
    wiced_result_t status = WICED_BT_SUCCESS;

    WICED_BT_TRACE( "Disconnecting Connection with Handle %d\n\r", handle );
    if (handle == 0)
    {
        status = WICED_BT_ERROR;
    }
    else
    {
        status = wiced_bt_a2dp_source_disconnect(handle);
        av_app_cb.state = AV_STATE_DISCONNECTING;
    }

    return status;
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_stream_config
 ******************************************************************************
 * Summary:
 *          Configures the A2DP Stream
 *
 * Parameters:
 *          sf: sample frequency
 *          chcfg: channel configuration.
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_command_stream_config(uint8_t sf, uint8_t chcfg)
{
    switch (sf)
    {
    case AUDIO_SF_16K:
        sf = A2D_SBC_IE_SAMP_FREQ_16;
        break;
    case AUDIO_SF_32K:
        sf = A2D_SBC_IE_SAMP_FREQ_32;
        break;
    case AUDIO_SF_44_1K:
        sf = A2D_SBC_IE_SAMP_FREQ_44;
        break;
    case AUDIO_SF_48K:
        sf = A2D_SBC_IE_SAMP_FREQ_48;
        break;
    default:
        sf = A2D_SBC_IE_SAMP_FREQ_44;
        break;
    }

    switch (chcfg)
    {
    case AUDIO_CHCFG_MONO:
        chcfg = A2D_SBC_IE_CH_MD_MONO;
        break;
    case AUDIO_CHCFG_STEREO:
        chcfg = A2D_SBC_IE_CH_MD_STEREO;
        break;
    case AUDIO_CHCFG_JOINT:
        chcfg = A2D_SBC_IE_CH_MD_JOINT;
        break;
    default:
        chcfg = A2D_SBC_IE_CH_MD_JOINT;
        break;
    }

    if (sf != bt_audio_config.default_codec_config.cie.sbc.samp_freq || chcfg != bt_audio_config.default_codec_config.cie.sbc.ch_mode)
    {
        a2dp_source_deinit_streaming();
        bt_audio_config.default_codec_config.cie.sbc.samp_freq = sf;
        bt_audio_config.default_codec_config.cie.sbc.ch_mode = chcfg;
        if (sf == A2D_SBC_IE_SAMP_FREQ_44)
        {
            bt_audio_config.default_codec_config.cie.sbc.max_bitpool = BT_AUDIO_A2DP_SBC_MAX_BITPOOL_44K;
        }
        else
        {
            bt_audio_config.default_codec_config.cie.sbc.max_bitpool = BT_AUDIO_A2DP_SBC_MAX_BITPOOL_48K;
        }
    }
    WICED_BT_TRACE( "Handle %d sf:%s chcfg:%s\n\r", handle,
                    get_audio_freq_name (sf), get_audio_chcfg_name (chcfg) );
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_stream_start
 ******************************************************************************
 * Summary:
 *          Initiates Stream Start
 *
 * Parameters:
 *          None
 *
 * Return:
 *          status: result of Stream Start API
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_command_stream_start()
{
    WICED_BT_TRACE( "Stream Start for Connection Handle %d\n\r", handle);

    if (handle == 0)
    {
        return WICED_BT_ERROR;
    }

    return wiced_bt_a2dp_source_start(handle, &bt_audio_config.default_codec_config);
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_stream_stop
 ******************************************************************************
 * Summary:
 *          Initiates Stream Suspend
 *
 * Parameters:
 *          None
 *
 * Return:
 *          status: result of Stream Suspend API
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_command_stream_stop()
{
    WICED_BT_TRACE( "Stream Stop for Connection Handle %d\n\r", handle);

    if (handle == 0)
    {
        return WICED_BT_ERROR;
    }

    return wiced_bt_a2dp_source_suspend(handle);
}

/******************************************************************************
 * Function Name: a2dp_source_write_eir
 *******************************************************************************
 * Summary:
 *          Prepare extended inquiry response data.  Current version publishes
 *          audio source services.
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 ******************************************************************************/
void a2dp_source_write_eir( void )
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t *p_tmp;
    uint8_t nb_uuid = 0;
    uint8_t length;

    /* Allocating a buffer from the public pool */
    pBuf = (uint8_t*) wiced_bt_get_buffer ( WICED_HS_EIR_BUF_MAX_SIZE);

    if (!pBuf)
    {
        WICED_BT_TRACE ("Buffer allocation for EIR Data failed \n");
        return;
    }

    WICED_BT_TRACE ("EIR allocated Buffer: %x\n", pBuf);
    p = pBuf;

    length = (uint8_t) strlen ((char*) a2dp_source_cfg_settings.device_name);

    WICED_BT_TRACE ("length %d\n", (uint8_t)length);
    UINT8_TO_STREAM (p, length + 1);
    UINT8_TO_STREAM (p, BT_EIR_COMPLETE_LOCAL_NAME_TYPE);
    memcpy (p, a2dp_source_cfg_settings.device_name, length);
    p += length;

    /* Add other BR/EDR UUIDs */
    p_tmp = p;
    p++;
    UINT8_TO_STREAM (p, BT_EIR_COMPLETE_16BITS_UUID_TYPE);
    UINT16_TO_STREAM (p, UUID_SERVCLASS_AUDIO_SOURCE);
    nb_uuid++;

    /* Now, we can update the UUID Tag's length */
    UINT8_TO_STREAM (p_tmp, (nb_uuid * LEN_UUID_16) + 1);

    /* Last Tag */
    UINT8_TO_STREAM (p, 0x00);

    /* print EIR data */
    WICED_BT_TRACE_ARRAY( ( uint8_t* )( pBuf+1 ), MIN( p-( uint8_t* )pBuf,100 ), "EIR :" );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    /* Allocated buffer not anymore needed. Free it */
    wiced_bt_free_buffer (pBuf);
}

/******************************************************************************
 * Function Name: a2dp_source_write_nvram
 *******************************************************************************
 * Summary:
 *          Write NVRAM function is called to store information in the NVRAM.
 *
 * Parameters:
 *          nvram_id: NVRAM Id
 *          data_len: Length of the data to be written
 *          p_data: Data to be written
 *
 * Return:
 *          Number of bytes written
 *
 *****************************************************************************/
uint16_t a2dp_source_write_nvram( int nvram_id, int data_len, void *p_data)
{
    wiced_result_t  result;
    uint16_t        bytes_written = 0;

    if (p_data != NULL)
    {
        bytes_written = wiced_hal_write_nvram( nvram_id, data_len, (uint8_t*)p_data, &result );
        WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\n", nvram_id, bytes_written, result);
    }

    return (bytes_written);
}


/******************************************************************************
 * Function Name: a2dp_source_read_nvram
 *******************************************************************************
 * Summary:
 *          Read data from the NVRAM and return in the passed buffer
 *
 * Parameters:
 *          nvram_id: NVRAM Id
 *          data_len: Length of the data to be read
 *          p_data: Data buffer pointer to hold read data
 *
 * Return:
 *          Number of bytes read
 *
 *****************************************************************************/
uint16_t a2dp_source_read_nvram( int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;
    wiced_result_t  result;

    if ((p_data != NULL) && (data_len >= sizeof(wiced_bt_device_link_keys_t)))
    {
        read_bytes = wiced_hal_read_nvram( nvram_id, sizeof(wiced_bt_device_link_keys_t), p_data, &result );
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n", nvram_id,
                       sizeof(wiced_bt_device_link_keys_t), read_bytes, result );
    }
    return (read_bytes);
}

/******************************************************************************
 * Function Name: a2dp_source_app_init()
 *******************************************************************************
 * Summary:
 *   This function handles application level initialization tasks and is
 *   called from the BT management callback once the Bluetooth Stack enabled
 *   event (BTM_ENABLED_EVT) is triggered
 *   This function is executed in the BTM_ENABLED_EVT management callback.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 ******************************************************************************/
void a2dp_source_app_init( void )
{
    wiced_result_t result = WICED_BT_SUCCESS;
    av_app_cb.state = AV_STATE_IDLE;

    /* Register with the A2DP source profile code */
    result = wiced_bt_a2dp_source_init( &bt_audio_config,
                                        a2dp_source_control_cback, p_default_heap);
    if (result == WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE( "A2DP Source initialized \n\r");
    }
    else
    {
        WICED_BT_TRACE( "A2DP Source initialization failed \n\r");
    }
}

/******************************************************************************
 * Function Name: a2dp_source_management_callback()
 *******************************************************************************
 * Summary:
 *   This is a Bluetooth stack event handler function to receive management
 *   events from the Bluetooth stack and process as per the application.
 *
 * Parameters:
 *   wiced_bt_management_evt_t event : BLE event code of one byte length
 *   wiced_bt_management_evt_data_t *p_event_data: Pointer to BTStack management
 *   event structures
 *
 * Return:
 *  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 ******************************************************************************/

wiced_result_t a2dp_source_management_callback( wiced_bt_management_evt_t event,
                                              wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_ble_advert_mode_t          *p_mode; /* Advertisement Mode */
    uint8_t                             *p_keys; /* Paired event Link keys */
    wiced_bt_device_address_t           bda = {0};
    wiced_result_t                      result = WICED_BT_SUCCESS;

    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;
    wiced_bt_dev_encryption_status_t   *p_encryption_status;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    int                                 pairing_result;
    const uint8_t                      *link_key;

    WICED_BT_TRACE("%s: %s\n", __FUNCTION__, get_bt_event_name(event));

    switch( event )
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL");
            break;
        }
        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            /* Bluetooth is enabled */
            wiced_bt_set_local_bdaddr((uint8_t *)bt_device_address, BLE_ADDR_PUBLIC);
            wiced_bt_dev_read_local_addr(bda);
            WICED_BT_TRACE("Bluetooth has been enabled");
            WICED_BT_TRACE("Local Bluetooth Address: %02X:%02X:%02X:%02X:%02X:%02X\n", bda[0], bda[1], bda[2],
                    bda[3], bda[4], bda[5] );            

            /* Set Discoverable */
            a2dp_source_bt_set_visibility (WICED_TRUE, WICED_TRUE);

            /* Enable pairing */
            wiced_bt_set_pairable_mode(WICED_TRUE, 0);

            a2dp_source_write_eir();

            /* create SDP records */
            wiced_bt_sdp_db_init((uint8_t*) a2dp_source_sdp_db,
                sizeof(a2dp_source_sdp_db));

           /* start the a2dp application */
           a2dp_source_app_init();
        } else
        {
            WICED_BT_TRACE("Bluetooth enabling is failed \n");
        }
        break;

    case BTM_DISABLED_EVT:
            WICED_BT_TRACE("Bluetooth has been disabled \n");
        break;

    case BTM_SECURITY_FAILED_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("Security failed: %d / %d\n", p_event_data->security_failed.status,
                                                       p_event_data->security_failed.hci_status);
        break;

    case BTM_PIN_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("Remote address= %B\n", p_event_data->pin_request.bd_addr);
        wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr, WICED_BT_SUCCESS,
                                                        WICED_PIN_CODE_LEN, (uint8_t *)&pincode[0]);
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* If this is just works pairing, accept.
         * Otherwise send event to the MCU to confirm the same value.
         */
        WICED_BT_TRACE("User Confirmation. BDA %B, Key %d \n", p_event_data->user_confirmation_request.bd_addr,
                                                   p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
        break;

    case BTM_PASSKEY_NOTIFICATION_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n",
                                   p_event_data->user_passkey_notification.bd_addr,
                                   p_event_data->user_passkey_notification.passkey);
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* Use the default security for BR/EDR*/
        WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %B\n",
                       p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
        p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap =
                                                        BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req =
                                                        BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
        p_event_data->pairing_io_capabilities_br_edr_request.oob_data = WICED_FALSE;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req =
                                                        BTM_AUTH_ALL_PROFILES_NO;
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* Use the default security for BLE */
        WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT bda %B\n",
                p_event_data->pairing_io_capabilities_ble_request.bd_addr);
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  =
                                                    BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
        p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req =
                                                    BTM_LE_AUTH_REQ_SC_MITM_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = MAX_KEY_SIZE;
        p_event_data->pairing_io_capabilities_ble_request.init_keys =
                                                    BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys =
                                                    BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        p_pairing_cmpl = &p_event_data->pairing_complete;
        if(p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
        {
            pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
        }
        else
        {
            pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
        }
        WICED_BT_TRACE("Pairing Result: %d\n", pairing_result );
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        p_encryption_status = &p_event_data->encryption_status;
        WICED_BT_TRACE("Encryption Status:(%B) res:%d\n", p_encryption_status->bd_addr,
                                                           p_encryption_status->result );
        break;

    case BTM_SECURITY_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("Security Request Event \n");
        wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT - dev: [%B]  Len:%d\n",
                             p_event_data->paired_device_link_keys_update.bd_addr,
                             sizeof(wiced_bt_device_link_keys_t));

        /* This application supports a single paired host, we can save keys
         * under the same NVRAM ID overwriting previous pairing if any */
        a2dp_source_write_nvram(A2DP_SOURCE_NVRAM_ID, sizeof(wiced_bt_device_link_keys_t), &p_event_data->paired_device_link_keys_update);

        link_key = p_event_data->paired_device_link_keys_update.key_data.br_edr_key;
        WICED_BT_TRACE(
                " LinkKey:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                link_key[0], link_key[1], link_key[2], link_key[3], link_key[4],
                link_key[5], link_key[6], link_key[7], link_key[8], link_key[9],
                link_key[10], link_key[11], link_key[12], link_key[13],
                link_key[14], link_key[15]);
         break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT - dev: [%B]  Len:%d\n",
                            p_event_data->paired_device_link_keys_request.bd_addr,
                            sizeof(wiced_bt_device_link_keys_t));
        /* read existing key from the NVRAM  */
        if (a2dp_source_read_nvram (A2DP_SOURCE_NVRAM_ID,
                  &p_event_data->paired_device_link_keys_request,
                  sizeof(wiced_bt_device_link_keys_t)) != 0)
        {
            result = WICED_BT_SUCCESS;
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* save keys to NVRAM */
        p_keys = (uint8_t*) &p_event_data->local_identity_keys_update;
        WICED_BT_TRACE("    result: %d \n", result);
        break;

    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* read keys from NVRAM */
        p_keys = (uint8_t*) &p_event_data->local_identity_keys_request;
        break;

    case BTM_POWER_MANAGEMENT_STATUS_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
        WICED_BT_TRACE("Power mgmt status event: bd ( %B ) status:%d hci_status:%d\n",
                        p_power_mgmt_notification->bd_addr, p_power_mgmt_notification->status,
                        p_power_mgmt_notification->hci_status);
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        p_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE("Advertisement State Change: %d\n", *p_mode);
        break;

    case BTM_BLE_CONNECTION_PARAM_UPDATE:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* Connection parameters updated */
        if (WICED_SUCCESS == p_event_data->ble_connection_param_update.status)
        {
            WICED_BT_TRACE("Supervision Time Out = %d\n",
                    (p_event_data->ble_connection_param_update.supervision_timeout * 10));
        }
        break;

    default:
        WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));
        break;
    }
    return result;
}
/* [] END OF FILE */
