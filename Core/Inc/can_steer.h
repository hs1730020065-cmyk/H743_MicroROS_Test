/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can_steer.h
  * @brief   EPS 转向控制接口。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef CAN_STEER_H
#define CAN_STEER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fdcan.h"
#include <stdint.h>

/* 下发 EPS 目标转向角，单位：度。 */
HAL_StatusTypeDef EpsSteer_SendTargetAngle(int16_t angle_deg);
/* 下发 EPS 手动模式控制帧。 */
HAL_StatusTypeDef EpsSteer_SetManualMode(void);
/* 将 EPS 目标转向角设置为 0 度。 */
HAL_StatusTypeDef EpsSteer_SetZero(void);

/* EPS 反馈帧 ID (0x18F)。 */
#define EPS_FEEDBACK_ID  0x18FU

/* 从 0x18F 反馈帧中解析实际转向角，单位：度。 */
float EpsSteer_ParseFeedback(const uint8_t data[8]);

/* EPS 最近一次反馈的实际转向角，单位：度，供底盘解算使用。 */
extern volatile float g_eps_actual_angle_deg;
/* EPS 反馈帧接收计数，用于诊断。 */
extern volatile uint32_t g_eps_feedback_rx_count;
/* EPS 反馈帧最近一次接收时的系统 tick。 */
extern volatile uint32_t g_eps_feedback_last_tick;

#ifdef __cplusplus
}
#endif

#endif /* CAN_STEER_H */
