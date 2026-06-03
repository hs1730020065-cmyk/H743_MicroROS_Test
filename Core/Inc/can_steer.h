/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can_steer.h
  * @brief   Minimal EPS steering control interface.
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

HAL_StatusTypeDef EpsSteer_SendTargetAngle(int16_t angle_deg);
HAL_StatusTypeDef EpsSteer_SetManualMode(void);
HAL_StatusTypeDef EpsSteer_SetZero(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_STEER_H */
