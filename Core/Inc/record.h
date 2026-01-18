/*
 * record.h
 *
 *  Created on: Sep 13, 2022
 *      Author: HP
 *  Modified: Added streaming support - optimized version
 */

#ifndef RECORD_H_
#define RECORD_H_

#include "main.h"
#include "stm32746g_discovery_audio.h"
#include "fatfs.h"
#include "FreeRTOS.h"
#include "queue.h"

/*******************************************************************************
                            Defines
*******************************************************************************/
/* Recording file name */
#define REC_WAVE_NAME "WAVE.WAV"

/* Audio parameters */
#ifndef DEFAULT_AUDIO_IN_FREQ
#define DEFAULT_AUDIO_IN_FREQ               16000
#endif

#ifndef DEFAULT_AUDIO_IN_BIT_RESOLUTION
#define DEFAULT_AUDIO_IN_BIT_RESOLUTION     16
#endif

#ifndef DEFAULT_AUDIO_IN_CHANNEL_NBR
#define DEFAULT_AUDIO_IN_CHANNEL_NBR        1
#endif

/* Max recording length in bytes (60 seconds) */
#ifndef REC_SAMPLE_LENGTH
#define REC_SAMPLE_LENGTH                   (DEFAULT_AUDIO_IN_FREQ * DEFAULT_AUDIO_IN_CHANNEL_NBR * 2 * 60)
#endif

/* Buffer states */
#define BUFFER_EMPTY                        0
#define BUFFER_FULL                         1

/* Notification bits for tasks */
#define NOTIFY_HALF_TRANSFER                (1 << 0)
#define NOTIFY_TRANSFER_COMPLETE            (1 << 1)

#define AUDIO_IN_PCM_BUFFER_SIZE 4096

/*******************************************************************************
                            Types
*******************************************************************************/
typedef enum
{
    AUDIO_ERROR_NONE = 0,
    AUDIO_ERROR_IO,
    AUDIO_ERROR_EOF,
    AUDIO_ERROR_INVALID_VALUE,
} AUDIO_ErrorTypeDef;

typedef struct
{
    uint32_t SampleRate;
    uint32_t NbrChannels;
    uint32_t FileSize;
    uint32_t ByteRate;
    uint16_t BlockAlign;
    uint16_t BitPerSample;
    uint16_t SubChunk1Size;
} WAVE_FormatTypeDef;

/* Audio buffer control structure */
typedef struct
{
    uint16_t pcm_buff[AUDIO_IN_PCM_BUFFER_SIZE * 2] __attribute__((aligned(4)));
    uint32_t fptr;
    uint32_t pcm_ptr;
    uint32_t offset;
    uint32_t wr_state;
} AUDIO_IN_BufferTypeDef;

/* Audio mode enumeration */
typedef enum {
    AUDIO_MODE_IDLE = 0,
    AUDIO_MODE_RECORD,
    AUDIO_MODE_STREAM,
    AUDIO_MODE_BOTH
} AudioMode_t;

/*******************************************************************************
                            Function Prototypes
*******************************************************************************/

/* Configuration */
void Audio_SetStreamTaskHandle(TaskHandle_t handle);
void Audio_SetRecordTaskHandle(TaskHandle_t handle);
uint8_t Audio_GetMode(void);

/* Get PCM buffer pointer (for tasks to read) */
uint16_t* Audio_GetPcmBuffer(void);
uint32_t Audio_GetCurrentOffset(void);

/* Record functions */
AUDIO_ErrorTypeDef recordStart(void);
AUDIO_ErrorTypeDef recordProcess(void);
AUDIO_ErrorTypeDef recordStop(void);

/* Stream functions */
AUDIO_ErrorTypeDef streamStart(void);
AUDIO_ErrorTypeDef streamStop(void);

/* Record + Stream functions */
AUDIO_ErrorTypeDef recordAndStreamStart(void);
AUDIO_ErrorTypeDef recordAndStreamStop(void);

#endif /* RECORD_H_ */
