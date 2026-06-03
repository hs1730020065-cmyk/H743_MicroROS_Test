/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can_motor.h
  * @brief   Minimal FDCAN motor control interface.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void CanMotor_Init(void);
HAL_StatusTypeDef CanMotor_SendRawSpeed(uint16_t raw_speed);
HAL_StatusTypeDef CanMotor_Stop(void);
HAL_StatusTypeDef CanMotor_Forward(uint16_t speed_abs);
HAL_StatusTypeDef CanMotor_Reverse(uint16_t speed_abs);

#ifdef __cplusplus
}
#endif

#endif /* CAN_MOTOR_H */
