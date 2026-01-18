/*
 * record.c
 *
 *  Created on: Sep 13, 2022
 *      Author: HP
 *  Modified: Added streaming support - optimized version
 *
 *  Optimization:
 *  - Use Task Notification instead of Queue (faster, less RAM)
 *  - No data copy in ISR (just notify)
 *  - Tasks read directly from shared buffer
 */

#include "record.h"

/*******************************************************************************
                            Defines
*******************************************************************************/
#define BUF_SIZE AUDIO_IN_PCM_BUFFER_SIZE

/*******************************************************************************
                            Private Variables
*******************************************************************************/
/* Buffer for SD card writing (downsampled) */
uint16_t buffer[BUF_SIZE];
uint16_t buffer_size = BUF_SIZE;

uint8_t pHeaderBuff[44];

/* Main audio buffer - shared between DMA, Record Task, Stream Task */
static AUDIO_IN_BufferTypeDef BufferCtl;
static __IO uint32_t uwVolume = 100;
WAVE_FormatTypeDef WaveFormat;

/* Task handles for notification */
static TaskHandle_t xStreamTaskHandle = NULL;
static TaskHandle_t xRecordTaskHandle = NULL;

/* Current audio mode */
static volatile AudioMode_t currentMode = AUDIO_MODE_IDLE;

/* Flags for enabling features */
static volatile uint8_t recordEnabled = 0;
static volatile uint8_t streamEnabled = 0;

/*******************************************************************************
                            Static Functions
*******************************************************************************/

static uint32_t WavProcess_HeaderInit(uint8_t* pHeader, WAVE_FormatTypeDef* pWaveFormatStruct)
{
    pHeader[0] = 'R'; pHeader[1] = 'I'; pHeader[2] = 'F'; pHeader[3] = 'F';
    pHeader[4] = 0x00; pHeader[5] = 0x4C; pHeader[6] = 0x1D; pHeader[7] = 0x00;
    pHeader[8] = 'W'; pHeader[9] = 'A'; pHeader[10] = 'V'; pHeader[11] = 'E';
    pHeader[12] = 'f'; pHeader[13] = 'm'; pHeader[14] = 't'; pHeader[15] = ' ';
    pHeader[16] = 0x10; pHeader[17] = 0x00; pHeader[18] = 0x00; pHeader[19] = 0x00;
    pHeader[20] = 0x01; pHeader[21] = 0x00;
    pHeader[22] = pWaveFormatStruct->NbrChannels; pHeader[23] = 0x00;
    pHeader[24] = (uint8_t)((pWaveFormatStruct->SampleRate & 0xFF));
    pHeader[25] = (uint8_t)((pWaveFormatStruct->SampleRate >> 8) & 0xFF);
    pHeader[26] = (uint8_t)((pWaveFormatStruct->SampleRate >> 16) & 0xFF);
    pHeader[27] = (uint8_t)((pWaveFormatStruct->SampleRate >> 24) & 0xFF);
    pHeader[28] = (uint8_t)((pWaveFormatStruct->ByteRate & 0xFF));
    pHeader[29] = (uint8_t)((pWaveFormatStruct->ByteRate >> 8) & 0xFF);
    pHeader[30] = (uint8_t)((pWaveFormatStruct->ByteRate >> 16) & 0xFF);
    pHeader[31] = (uint8_t)((pWaveFormatStruct->ByteRate >> 24) & 0xFF);
    pHeader[32] = pWaveFormatStruct->BlockAlign; pHeader[33] = 0x00;
    pHeader[34] = pWaveFormatStruct->BitPerSample; pHeader[35] = 0x00;
    pHeader[36] = 'd'; pHeader[37] = 'a'; pHeader[38] = 't'; pHeader[39] = 'a';
    pHeader[40] = 0x00; pHeader[41] = 0x4C; pHeader[42] = 0x1D; pHeader[43] = 0x00;
    return 0;
}

static uint32_t WavProcess_EncInit(uint32_t Freq, uint8_t *pHeader)
{
    WaveFormat.SampleRate = Freq;
    WaveFormat.NbrChannels = 1;
    WaveFormat.BitPerSample = 16;
    WaveFormat.FileSize = 0x001D4C00;
    WaveFormat.SubChunk1Size = 44;
    WaveFormat.ByteRate = (WaveFormat.SampleRate * (WaveFormat.BitPerSample/8) * WaveFormat.NbrChannels);
    WaveFormat.BlockAlign = WaveFormat.NbrChannels * (WaveFormat.BitPerSample/8);

    if(WavProcess_HeaderInit(pHeader, &WaveFormat)) {
        return 1;
    }
    return 0;
}

static uint32_t WavProcess_HeaderUpdate(uint8_t* pHeader, WAVE_FormatTypeDef* pWaveFormatStruct)
{
    pHeader[4] = (uint8_t)(BufferCtl.fptr);
    pHeader[5] = (uint8_t)(BufferCtl.fptr >> 8);
    pHeader[6] = (uint8_t)(BufferCtl.fptr >> 16);
    pHeader[7] = (uint8_t)(BufferCtl.fptr >> 24);

    BufferCtl.fptr -= 44;
    pHeader[40] = (uint8_t)(BufferCtl.fptr);
    pHeader[41] = (uint8_t)(BufferCtl.fptr >> 8);
    pHeader[42] = (uint8_t)(BufferCtl.fptr >> 16);
    pHeader[43] = (uint8_t)(BufferCtl.fptr >> 24);

    return 0;
}

/*******************************************************************************
                            Public Functions - Configuration
*******************************************************************************/

void Audio_SetStreamTaskHandle(TaskHandle_t handle)
{
    xStreamTaskHandle = handle;
}

void Audio_SetRecordTaskHandle(TaskHandle_t handle)
{
    xRecordTaskHandle = handle;
}

uint8_t Audio_GetMode(void)
{
    return (uint8_t)currentMode;
}

/* Get pointer to PCM buffer for tasks to read directly */
uint16_t* Audio_GetPcmBuffer(void)
{
    return BufferCtl.pcm_buff;
}

/* Get current offset (0 = first half, AUDIO_IN_PCM_BUFFER_SIZE = second half) */
uint32_t Audio_GetCurrentOffset(void)
{
    return BufferCtl.offset;
}

/*******************************************************************************
                            Public Functions - Record Only
*******************************************************************************/

AUDIO_ErrorTypeDef recordStart(void)
{
    FRESULT res;
    uint32_t byteswritten = 0;
    uwVolume = 100;

    res = f_open(&SDFile, REC_WAVE_NAME, FA_CREATE_ALWAYS | FA_WRITE);
    if(res != FR_OK) {
        return AUDIO_ERROR_IO;
    }

    WavProcess_EncInit(DEFAULT_AUDIO_IN_FREQ, pHeaderBuff);

    if(f_write(&SDFile, pHeaderBuff, 44, (void*)&byteswritten) == FR_OK) {
        if(byteswritten != 0) {
            BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, 1);
            BSP_AUDIO_IN_Record((uint16_t*)&BufferCtl.pcm_buff[0], AUDIO_IN_PCM_BUFFER_SIZE);

            BufferCtl.fptr = byteswritten;
            BufferCtl.pcm_ptr = 0;
            BufferCtl.offset = 0;
            BufferCtl.wr_state = BUFFER_EMPTY;

            recordEnabled = 1;
            streamEnabled = 0;
            currentMode = AUDIO_MODE_RECORD;

            HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, SET);
        }
    }

    return AUDIO_ERROR_NONE;
}

AUDIO_ErrorTypeDef recordProcess(void)
{
    uint32_t elapsed_time;
    static uint32_t prev_elapsed_time = 0xFFFFFFFF;
    FRESULT res;
    uint32_t byteswritten = 0;

    if (!recordEnabled) {
        return AUDIO_ERROR_NONE;
    }

    if(BufferCtl.fptr >= REC_SAMPLE_LENGTH) {
        return AUDIO_ERROR_EOF;
    }

    if (BufferCtl.wr_state == BUFFER_FULL) {
        /* Downsample and copy to buffer */
        for(int i = 0; i < AUDIO_IN_PCM_BUFFER_SIZE/2; i++) {
            buffer[i] = BufferCtl.pcm_buff[BufferCtl.offset + i*4];
        }

        res = f_write(&SDFile, (uint16_t*)(buffer), AUDIO_IN_PCM_BUFFER_SIZE/2, (void*)&byteswritten);
        if(res != FR_OK) {
            return AUDIO_ERROR_IO;
        }
        BufferCtl.fptr += byteswritten;
        BufferCtl.wr_state = BUFFER_EMPTY;
    }

    elapsed_time = BufferCtl.fptr / (DEFAULT_AUDIO_IN_FREQ * DEFAULT_AUDIO_IN_CHANNEL_NBR * 2);
    if(prev_elapsed_time != elapsed_time) {
        prev_elapsed_time = elapsed_time;
    }

    return AUDIO_ERROR_NONE;
}

AUDIO_ErrorTypeDef recordStop(void)
{
    FRESULT res;
    uint32_t byteswritten = 0;

    if (!recordEnabled && currentMode != AUDIO_MODE_BOTH) {
        return AUDIO_ERROR_NONE;
    }

    BSP_AUDIO_IN_Stop(CODEC_PDWN_SW);
    HAL_Delay(150);

    res = f_lseek(&SDFile, 0);
    if(res == FR_OK) {
        WavProcess_HeaderUpdate(pHeaderBuff, &WaveFormat);
        f_write(&SDFile, pHeaderBuff, sizeof(WAVE_FormatTypeDef), (void*)&byteswritten);
    }

    f_close(&SDFile);

    recordEnabled = 0;
    streamEnabled = 0;
    currentMode = AUDIO_MODE_IDLE;

    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, RESET);

    return AUDIO_ERROR_NONE;
}

/*******************************************************************************
                            Public Functions - Stream Only
*******************************************************************************/

AUDIO_ErrorTypeDef streamStart(void)
{
    if (xStreamTaskHandle == NULL) {
        return AUDIO_ERROR_IO;
    }

    uwVolume = 100;

    BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, 1);
    BSP_AUDIO_IN_Record((uint16_t*)&BufferCtl.pcm_buff[0], AUDIO_IN_PCM_BUFFER_SIZE);

    BufferCtl.fptr = 0;
    BufferCtl.pcm_ptr = 0;
    BufferCtl.offset = 0;
    BufferCtl.wr_state = BUFFER_EMPTY;

    recordEnabled = 0;
    streamEnabled = 1;
    currentMode = AUDIO_MODE_STREAM;

    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, SET);

    return AUDIO_ERROR_NONE;
}

AUDIO_ErrorTypeDef streamStop(void)
{
    if (!streamEnabled) {
        return AUDIO_ERROR_NONE;
    }

    BSP_AUDIO_IN_Stop(CODEC_PDWN_SW);
    HAL_Delay(50);

    streamEnabled = 0;
    currentMode = AUDIO_MODE_IDLE;

    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, RESET);

    return AUDIO_ERROR_NONE;
}

/*******************************************************************************
                            Public Functions - Record + Stream
*******************************************************************************/

AUDIO_ErrorTypeDef recordAndStreamStart(void)
{
    FRESULT res;
    uint32_t byteswritten = 0;

    if (xStreamTaskHandle == NULL) {
        return AUDIO_ERROR_IO;
    }

    uwVolume = 100;

    res = f_open(&SDFile, REC_WAVE_NAME, FA_CREATE_ALWAYS | FA_WRITE);
    if(res != FR_OK) {
        return AUDIO_ERROR_IO;
    }

    WavProcess_EncInit(DEFAULT_AUDIO_IN_FREQ, pHeaderBuff);

    if(f_write(&SDFile, pHeaderBuff, 44, (void*)&byteswritten) == FR_OK) {
        if(byteswritten != 0) {
            BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, 1);
            BSP_AUDIO_IN_Record((uint16_t*)&BufferCtl.pcm_buff[0], AUDIO_IN_PCM_BUFFER_SIZE);

            BufferCtl.fptr = byteswritten;
            BufferCtl.pcm_ptr = 0;
            BufferCtl.offset = 0;
            BufferCtl.wr_state = BUFFER_EMPTY;

            recordEnabled = 1;
            streamEnabled = 1;
            currentMode = AUDIO_MODE_BOTH;

            HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, SET);
        }
    }

    return AUDIO_ERROR_NONE;
}

AUDIO_ErrorTypeDef recordAndStreamStop(void)
{
    return recordStop();
}

/*******************************************************************************
                            DMA Callbacks - Minimal work, just notify tasks
*******************************************************************************/

void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    BufferCtl.wr_state = BUFFER_FULL;
    BufferCtl.offset = 0;

    /* Notify Stream Task */
    if (streamEnabled && xStreamTaskHandle != NULL) {
        xTaskNotifyFromISR(xStreamTaskHandle, NOTIFY_HALF_TRANSFER, eSetBits, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    BufferCtl.wr_state = BUFFER_FULL;
    BufferCtl.offset = AUDIO_IN_PCM_BUFFER_SIZE;

    /* Notify Stream Task */
    if (streamEnabled && xStreamTaskHandle != NULL) {
        xTaskNotifyFromISR(xStreamTaskHandle, NOTIFY_TRANSFER_COMPLETE, eSetBits, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void BSP_AUDIO_IN_Error_CallBack(void)
{
    recordEnabled = 0;
    streamEnabled = 0;
    currentMode = AUDIO_MODE_IDLE;
}
