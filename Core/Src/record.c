/*
 * record.c - Hardware Test Version with Debug Output
 *
 * Test sequence:
 * 1. Press button -> Start stream
 * 2. See "DMA H" and "DMA F" messages for each DMA callback
 * 3. See "TX:xxx" for each UART packet sent
 * 4. Press button again -> Stop, see statistics
 */

#include "record.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"

/*============================================================================
 * RING BUFFER
 *============================================================================*/
#define RING_BUFFER_MASK    (RING_BUFFER_SIZE - 1)

typedef struct {
    volatile uint16_t buffer[RING_BUFFER_SIZE];
    volatile uint32_t write_idx;
    volatile uint32_t read_idx;
} RingBuffer_t;

/*============================================================================
 * PRIVATE VARIABLES
 *============================================================================*/
static AUDIO_IN_BufferTypeDef BufferCtl;
static RingBuffer_t g_ring;

/* UART TX - single buffer for simplicity */
static uint8_t g_tx_buffer[520];  /* 4 header + 512 data max */
static volatile bool g_tx_busy = false;

/* Handles */
static TaskHandle_t xStreamTaskHandle = NULL;
static UART_HandleTypeDef *g_huart = NULL;

/* State */
static volatile AudioMode_t currentMode = AUDIO_MODE_IDLE;
static volatile uint8_t streamEnabled = 0;
static volatile uint8_t recordEnabled = 0;

/* Stats */
static StreamStats_t g_stats;

/* WAV */
static uint8_t pHeaderBuff[44];
static WAVE_FormatTypeDef WaveFormat;
static uint16_t sd_buffer[AUDIO_IN_PCM_BUFFER_SIZE / 2];

/*============================================================================
 * RING BUFFER FUNCTIONS
 *============================================================================*/
static inline uint32_t Ring_Count(void) {
    return (g_ring.write_idx - g_ring.read_idx) & RING_BUFFER_MASK;
}

static inline uint32_t Ring_Free(void) {
    return RING_BUFFER_SIZE - 1 - Ring_Count();
}

static uint32_t Ring_Write(const uint16_t *data, uint32_t count)
{
    uint32_t free = Ring_Free();
    if (count > free) {
        count = free;
        g_stats.ring_overflows++;
    }

    uint32_t wr = g_ring.write_idx;
    for (uint32_t i = 0; i < count; i++) {
        g_ring.buffer[wr & RING_BUFFER_MASK] = data[i];
        wr++;
    }
    __DMB();
    g_ring.write_idx = wr;
    return count;
}

static uint32_t Ring_Read(uint16_t *data, uint32_t count)
{
    uint32_t avail = Ring_Count();
    if (count > avail) count = avail;
    if (count == 0) return 0;

    uint32_t rd = g_ring.read_idx;
    for (uint32_t i = 0; i < count; i++) {
        data[i] = g_ring.buffer[rd & RING_BUFFER_MASK];
        rd++;
    }
    __DMB();
    g_ring.read_idx = rd;
    return count;
}

static void Ring_Reset(void) {
    g_ring.write_idx = 0;
    g_ring.read_idx = 0;
}

/*============================================================================
 * WAV FUNCTIONS
 *============================================================================*/
static void WavHeader_Init(uint8_t* p, WAVE_FormatTypeDef* w)
{
    memcpy(p, "RIFF", 4);
    p[4] = 0; p[5] = 0x4C; p[6] = 0x1D; p[7] = 0;
    memcpy(p+8, "WAVEfmt ", 8);
    p[16] = 0x10; p[17] = 0; p[18] = 0; p[19] = 0;
    p[20] = 1; p[21] = 0;
    p[22] = w->NbrChannels; p[23] = 0;
    p[24] = w->SampleRate & 0xFF;
    p[25] = (w->SampleRate >> 8) & 0xFF;
    p[26] = (w->SampleRate >> 16) & 0xFF;
    p[27] = (w->SampleRate >> 24) & 0xFF;
    p[28] = w->ByteRate & 0xFF;
    p[29] = (w->ByteRate >> 8) & 0xFF;
    p[30] = (w->ByteRate >> 16) & 0xFF;
    p[31] = (w->ByteRate >> 24) & 0xFF;
    p[32] = w->BlockAlign; p[33] = 0;
    p[34] = w->BitPerSample; p[35] = 0;
    memcpy(p+36, "data", 4);
    p[40] = 0; p[41] = 0x4C; p[42] = 0x1D; p[43] = 0;
}

static void WavHeader_Update(uint8_t* p)
{
    uint32_t fileSize = BufferCtl.fptr;
    p[4] = fileSize & 0xFF;
    p[5] = (fileSize >> 8) & 0xFF;
    p[6] = (fileSize >> 16) & 0xFF;
    p[7] = (fileSize >> 24) & 0xFF;

    uint32_t dataSize = fileSize - 44;
    p[40] = dataSize & 0xFF;
    p[41] = (dataSize >> 8) & 0xFF;
    p[42] = (dataSize >> 16) & 0xFF;
    p[43] = (dataSize >> 24) & 0xFF;
}

/*============================================================================
 * PUBLIC API - Config
 *============================================================================*/
void Audio_SetStreamTaskHandle(TaskHandle_t handle) {
    xStreamTaskHandle = handle;
}

void Audio_SetUartHandle(UART_HandleTypeDef *huart) {
    g_huart = huart;
}

AudioMode_t Audio_GetMode(void) {
    return currentMode;
}

void Audio_GetStats(StreamStats_t *stats) {
    if (stats) {
        taskENTER_CRITICAL();
        memcpy(stats, &g_stats, sizeof(StreamStats_t));
        taskEXIT_CRITICAL();
    }
}

void Audio_ResetStats(void) {
    memset(&g_stats, 0, sizeof(StreamStats_t));
}

uint8_t Audio_GetRingBufferLevel(void) {
    return (uint8_t)((Ring_Count() * 100) / RING_BUFFER_SIZE);
}

/*============================================================================
 * STREAMING
 *============================================================================*/
AUDIO_ErrorTypeDef streamStart(void)
{
    if (g_huart == NULL) g_huart = &huart6;
    if (xStreamTaskHandle == NULL) return AUDIO_ERROR_NOT_INITIALIZED;

    Ring_Reset();
    g_tx_busy = false;
    Audio_ResetStats();

    /* Init audio */
    BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
    BSP_AUDIO_IN_Record((uint16_t*)BufferCtl.pcm_buff, AUDIO_IN_PCM_BUFFER_SIZE);

    BufferCtl.offset = 0;
    BufferCtl.wr_state = BUFFER_EMPTY;

    streamEnabled = 1;
    recordEnabled = 0;
    currentMode = AUDIO_MODE_STREAM;

    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, SET);

    return AUDIO_ERROR_NONE;
}

AUDIO_ErrorTypeDef streamStop(void)
{
    if (!streamEnabled) return AUDIO_ERROR_NONE;

    BSP_AUDIO_IN_Stop(CODEC_PDWN_SW);
    vTaskDelay(50);

    if (g_tx_busy && g_huart) {
        HAL_UART_AbortTransmit(g_huart);
        g_tx_busy = false;
    }

    streamEnabled = 0;
    currentMode = AUDIO_MODE_IDLE;

    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, RESET);

    return AUDIO_ERROR_NONE;
}

/**
 * @brief Process stream - called from stream task
 */
void streamProcess(void)
{
    static uint16_t temp_samples[STREAM_PACKET_MAX_SAMPLES];

    if (!streamEnabled || g_huart == NULL) return;
    if (g_tx_busy) return;

    uint32_t avail = Ring_Count();
    if (avail < STREAM_PACKET_MAX_SAMPLES) return;

    /* Read from ring buffer */
    uint32_t count = Ring_Read(temp_samples, STREAM_PACKET_MAX_SAMPLES);
    if (count == 0) return;

    /* Build packet */
    uint16_t audio_bytes = count * 2;
    uint16_t pkt_size = 0;

    /* Sync word */
    g_tx_buffer[pkt_size++] = 0xAA;
    g_tx_buffer[pkt_size++] = 0x55;

    /* Length (big-endian) */
    g_tx_buffer[pkt_size++] = (audio_bytes >> 8) & 0xFF;
    g_tx_buffer[pkt_size++] = audio_bytes & 0xFF;

    /* Audio data */
    memcpy(&g_tx_buffer[pkt_size], temp_samples, audio_bytes);
    pkt_size += audio_bytes;

    /* Send via UART DMA */
    g_tx_busy = true;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(g_huart, g_tx_buffer, pkt_size);

    if (status != HAL_OK) {
        g_tx_busy = false;
        g_stats.uart_errors++;
        return;
    }

    g_stats.samples_streamed += count;
    g_stats.packets_sent++;
}

/*============================================================================
 * RECORDING
 *============================================================================*/
AUDIO_ErrorTypeDef recordStart(void)
{
    FRESULT res;
    uint32_t bw;

    res = f_open(&SDFile, REC_WAVE_NAME, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) return AUDIO_ERROR_IO;

    WaveFormat.SampleRate = DEFAULT_AUDIO_IN_FREQ;
    WaveFormat.NbrChannels = 1;
    WaveFormat.BitPerSample = 16;
    WaveFormat.ByteRate = DEFAULT_AUDIO_IN_FREQ * 2;
    WaveFormat.BlockAlign = 2;

    WavHeader_Init(pHeaderBuff, &WaveFormat);

    if (f_write(&SDFile, pHeaderBuff, 44, (void*)&bw) != FR_OK || bw == 0) {
        f_close(&SDFile);
        return AUDIO_ERROR_IO;
    }

    BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
    BSP_AUDIO_IN_Record((uint16_t*)BufferCtl.pcm_buff, AUDIO_IN_PCM_BUFFER_SIZE);

    BufferCtl.fptr = bw;
    BufferCtl.offset = 0;
    BufferCtl.wr_state = BUFFER_EMPTY;

    recordEnabled = 1;
    streamEnabled = 0;
    currentMode = AUDIO_MODE_RECORD;

    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, SET);

    return AUDIO_ERROR_NONE;
}

AUDIO_ErrorTypeDef recordProcess(void)
{
    FRESULT res;
    uint32_t bw;

    if (!recordEnabled) return AUDIO_ERROR_NONE;
    if (BufferCtl.fptr >= REC_SAMPLE_LENGTH) return AUDIO_ERROR_EOF;

    if (BufferCtl.wr_state == BUFFER_FULL) {
        uint16_t half = AUDIO_IN_PCM_BUFFER_SIZE / 2;
        memcpy(sd_buffer, &BufferCtl.pcm_buff[BufferCtl.offset], half * 2);

        res = f_write(&SDFile, sd_buffer, half * 2, (void*)&bw);
        if (res != FR_OK) return AUDIO_ERROR_IO;

        BufferCtl.fptr += bw;
        BufferCtl.wr_state = BUFFER_EMPTY;
    }

    return AUDIO_ERROR_NONE;
}

AUDIO_ErrorTypeDef recordStop(void)
{
    uint32_t bw;

    if (!recordEnabled && currentMode != AUDIO_MODE_BOTH) return AUDIO_ERROR_NONE;

    BSP_AUDIO_IN_Stop(CODEC_PDWN_SW);
    HAL_Delay(150);

    f_lseek(&SDFile, 0);
    WavHeader_Update(pHeaderBuff);
    f_write(&SDFile, pHeaderBuff, 44, (void*)&bw);
    f_close(&SDFile);

    recordEnabled = 0;
    streamEnabled = 0;
    currentMode = AUDIO_MODE_IDLE;

    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, RESET);

    return AUDIO_ERROR_NONE;
}

/*============================================================================
 * COMBINED STREAM + RECORD
 *============================================================================*/
AUDIO_ErrorTypeDef recordAndStreamStart(void)
{
    FRESULT res;
    uint32_t bw;

    if (g_huart == NULL) g_huart = &huart6;
    if (xStreamTaskHandle == NULL) return AUDIO_ERROR_NOT_INITIALIZED;

    Ring_Reset();
    g_tx_busy = false;
    Audio_ResetStats();

    /* Open file */
    res = f_open(&SDFile, REC_WAVE_NAME, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) return AUDIO_ERROR_IO;

    WaveFormat.SampleRate = DEFAULT_AUDIO_IN_FREQ;
    WaveFormat.NbrChannels = 1;
    WaveFormat.BitPerSample = 16;
    WaveFormat.ByteRate = DEFAULT_AUDIO_IN_FREQ * 2;
    WaveFormat.BlockAlign = 2;

    WavHeader_Init(pHeaderBuff, &WaveFormat);

    if (f_write(&SDFile, pHeaderBuff, 44, (void*)&bw) != FR_OK || bw == 0) {
        f_close(&SDFile);
        return AUDIO_ERROR_IO;
    }

    /* Init audio */
    BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
    BSP_AUDIO_IN_Record((uint16_t*)BufferCtl.pcm_buff, AUDIO_IN_PCM_BUFFER_SIZE);

    BufferCtl.fptr = bw;
    BufferCtl.offset = 0;
    BufferCtl.wr_state = BUFFER_EMPTY;

    recordEnabled = 1;
    streamEnabled = 1;
    currentMode = AUDIO_MODE_BOTH;

    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, SET);

    return AUDIO_ERROR_NONE;
}

AUDIO_ErrorTypeDef recordAndStreamStop(void)
{
    if (g_tx_busy && g_huart) {
        HAL_UART_AbortTransmit(g_huart);
        g_tx_busy = false;
    }
    streamEnabled = 0;
    return recordStop();
}

/*============================================================================
 * DMA CALLBACKS - Called from ISR
 *============================================================================*/
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
    BaseType_t woken = pdFALSE;

    BufferCtl.wr_state = BUFFER_FULL;
    BufferCtl.offset = 0;

    g_stats.samples_captured += AUDIO_IN_PCM_BUFFER_SIZE / 2;
    g_stats.dma_half_count++;

    /* Copy to ring buffer for streaming */
    if (streamEnabled) {
        Ring_Write(&BufferCtl.pcm_buff[0], AUDIO_IN_PCM_BUFFER_SIZE / 2);

        if (xStreamTaskHandle) {
            xTaskNotifyFromISR(xStreamTaskHandle, NOTIFY_HALF_TRANSFER, eSetBits, &woken);
        }
    }

    portYIELD_FROM_ISR(woken);
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
    BaseType_t woken = pdFALSE;

    BufferCtl.wr_state = BUFFER_FULL;
    BufferCtl.offset = AUDIO_IN_PCM_BUFFER_SIZE / 2;

    g_stats.samples_captured += AUDIO_IN_PCM_BUFFER_SIZE / 2;
    g_stats.dma_full_count++;

    /* Copy to ring buffer for streaming */
    if (streamEnabled) {
        Ring_Write(&BufferCtl.pcm_buff[AUDIO_IN_PCM_BUFFER_SIZE / 2], AUDIO_IN_PCM_BUFFER_SIZE / 2);

        if (xStreamTaskHandle) {
            xTaskNotifyFromISR(xStreamTaskHandle, NOTIFY_TRANSFER_COMPLETE, eSetBits, &woken);
        }
    }

    portYIELD_FROM_ISR(woken);
}

void BSP_AUDIO_IN_Error_CallBack(void)
{
    recordEnabled = 0;
    streamEnabled = 0;
    currentMode = AUDIO_MODE_IDLE;
}

/*============================================================================
 * UART CALLBACKS
 *============================================================================*/
void Audio_UART_TxCpltCallback(void)
{
    BaseType_t woken = pdFALSE;

    g_tx_busy = false;

    if (xStreamTaskHandle) {
        xTaskNotifyFromISR(xStreamTaskHandle, NOTIFY_UART_TX_COMPLETE, eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

void Audio_UART_ErrorCallback(void)
{
    g_stats.uart_errors++;
    g_tx_busy = false;

    if (g_huart) {
        HAL_UART_AbortTransmit(g_huart);
    }
}
