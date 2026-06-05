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

#ifdef __cplusplus
}
#endif

#endif /* CAN_STEER_H */
