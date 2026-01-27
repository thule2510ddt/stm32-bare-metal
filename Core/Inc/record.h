/*
 * record.h - Simple Test Version for Hardware Verification
 */

#ifndef RECORD_H_
#define RECORD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_audio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fatfs.h"
#include <stdbool.h>

/*============================================================================
 * CONFIGURATION
 *============================================================================*/

/* DMA Buffer - 256 samples total = 128 per half = 8ms latency @16kHz */
#define AUDIO_IN_PCM_BUFFER_SIZE        256U

/* Ring buffer - 2048 samples = 128ms buffer */
#define RING_BUFFER_SIZE                2048U

/* Stream packet */
#define STREAM_PACKET_SYNC              0xAA55U
#define STREAM_PACKET_MAX_SAMPLES       128U

/* Recording */
#define REC_WAVE_NAME                   "record.wav"
#define REC_SAMPLE_LENGTH               (DEFAULT_AUDIO_IN_FREQ * 60 * 2)

/*============================================================================
 * TYPES
 *============================================================================*/
typedef enum {
    AUDIO_ERROR_NONE = 0,
    AUDIO_ERROR_IO,
    AUDIO_ERROR_EOF,
    AUDIO_ERROR_INVALID_VALUE,
    AUDIO_ERROR_BUSY,
    AUDIO_ERROR_NOT_INITIALIZED
} AUDIO_ErrorTypeDef;

typedef enum {
    AUDIO_MODE_IDLE = 0,
    AUDIO_MODE_STREAM,
    AUDIO_MODE_RECORD,
    AUDIO_MODE_BOTH
} AudioMode_t;

typedef enum {
    BUFFER_EMPTY = 0,
    BUFFER_HALF_FULL,
    BUFFER_FULL
} BufferState_t;

/* Task notifications */
#define NOTIFY_HALF_TRANSFER        (1UL << 0)
#define NOTIFY_TRANSFER_COMPLETE    (1UL << 1)
#define NOTIFY_UART_TX_COMPLETE     (1UL << 2)
#define NOTIFY_STOP                 (1UL << 3)

/* Statistics */
typedef struct {
    uint32_t samples_captured;
    uint32_t samples_streamed;
    uint32_t packets_sent;
    uint32_t ring_overflows;
    uint32_t uart_errors;
    uint32_t dma_half_count;
    uint32_t dma_full_count;
} StreamStats_t;

typedef struct {
    uint32_t SampleRate;
    uint32_t FileSize;
    uint16_t NbrChannels;
    uint16_t BitPerSample;
    uint32_t ByteRate;
    uint16_t BlockAlign;
    uint32_t DataSize;
    uint32_t SubChunk1Size;
} WAVE_FormatTypeDef;

typedef struct {
    uint16_t pcm_buff[AUDIO_IN_PCM_BUFFER_SIZE];
    uint32_t fptr;
    uint32_t pcm_ptr;
    uint32_t offset;
    BufferState_t wr_state;
} AUDIO_IN_BufferTypeDef;

/*============================================================================
 * API
 *============================================================================*/
void Audio_SetStreamTaskHandle(TaskHandle_t handle);
void Audio_SetUartHandle(UART_HandleTypeDef *huart);
AudioMode_t Audio_GetMode(void);
void Audio_GetStats(StreamStats_t *stats);
void Audio_ResetStats(void);
uint8_t Audio_GetRingBufferLevel(void);

/* Streaming */
AUDIO_ErrorTypeDef streamStart(void);
AUDIO_ErrorTypeDef streamStop(void);
void streamProcess(void);

/* Recording */
AUDIO_ErrorTypeDef recordStart(void);
AUDIO_ErrorTypeDef recordProcess(void);
AUDIO_ErrorTypeDef recordStop(void);

/* Combined */
AUDIO_ErrorTypeDef recordAndStreamStart(void);
AUDIO_ErrorTypeDef recordAndStreamStop(void);

/* Callbacks */
void Audio_UART_TxCpltCallback(void);
void Audio_UART_ErrorCallback(void);

/* External */
extern UART_HandleTypeDef huart6;
extern FIL SDFile;

#ifdef __cplusplus
}
#endif

#endif /* RECORD_H_ */
