/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sbus.h
  * @brief   SBUS receiver parsing and manual control mapping.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef SBUS_H
#define SBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define SBUS_FRAME_SIZE        25U
#define SBUS_CHANNEL_COUNT     16U

#define SBUS_HEADER            0x0FU
#define SBUS_MID_VALUE         992
#define SBUS_MIN_VALUE         172
#define SBUS_MAX_VALUE         1811
#define SBUS_DEADZONE          50

typedef struct
{
  uint16_t ch[SBUS_CHANNEL_COUNT];

  uint8_t ch17;
  uint8_t ch18;
  uint8_t frame_lost;
  uint8_t failsafe;

  uint8_t frame_valid;
} SbusData_t;

typedef struct
{
  int16_t throttle;   /* -1000..1000, positive forward, negative reverse */
  int16_t steering;   /* -1000..1000, positive right, negative left */
  uint8_t manual_mode;
  uint8_t estop;
} SbusControl_t;

void SBUS_Init(void);
void SBUS_SerialAssistantTest_Init(void);
bool SBUS_TryParseFrame(const uint8_t frame[SBUS_FRAME_SIZE], SbusData_t *out);
void SBUS_ConvertToControl(const SbusData_t *sbus, SbusControl_t *ctrl);
const uint8_t *SBUS_GetRxBuffer(void);
uint32_t SBUS_GetRxFrameCount(void);
void SBUS_SendSerialAssistantReadyText(void);
void SBUS_SendSerialAssistantBadFrameText(void);
void SBUS_SendControlDirectionText(const SbusControl_t *ctrl);

void SBUS_TextCommandTest_Init(void);
bool SBUS_TextCommandTest_Process(SbusControl_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* SBUS_H */
