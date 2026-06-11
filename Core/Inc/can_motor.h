/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can_motor.h
  * @brief   FDCAN 后驱电机控制接口。
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

#define CAN_MOTOR_FEEDBACK_ID 0x186U

/* 初始化 FDCAN1 并刷新诊断状态。 */
void CanMotor_Init(void);
/* 读取并更新 FDCAN1 诊断信息。 */
void CanMotor_UpdateDiagnostics(void);
/* 直接发送电机协议原始速度值。 */
HAL_StatusTypeDef CanMotor_SendRawSpeed(uint16_t raw_speed);
/* 下发停止帧。 */
HAL_StatusTypeDef CanMotor_Stop(void);
/* 按绝对速度下发前进命令。 */
HAL_StatusTypeDef CanMotor_Forward(uint16_t speed_abs);
/* 按绝对速度下发后退命令。 */
HAL_StatusTypeDef CanMotor_Reverse(uint16_t speed_abs);

/* FDCAN1 启动返回状态。 */
extern volatile HAL_StatusTypeDef g_fdcan_start_status;
/* HAL 层最近一次 FDCAN 错误码。 */
extern volatile uint32_t g_fdcan_hal_error;
/* FDCAN 发送 FIFO 当前空闲数量。 */
extern volatile uint32_t g_fdcan_tx_fifo_free_level;
/* FDCAN 协议状态中的最近错误类型。 */
extern volatile uint32_t g_fdcan_last_error_code;
/* FDCAN 当前总线活动状态。 */
extern volatile uint32_t g_fdcan_activity;
/* FDCAN 是否处于错误被动状态。 */
extern volatile uint32_t g_fdcan_error_passive;
/* FDCAN 是否进入警告状态。 */
extern volatile uint32_t g_fdcan_warning;
/* FDCAN 是否进入 bus-off 状态。 */
extern volatile uint32_t g_fdcan_bus_off;
/* FDCAN 发送错误计数。 */
extern volatile uint32_t g_fdcan_tx_error_count;
/* FDCAN 接收错误计数。 */
extern volatile uint32_t g_fdcan_rx_error_count;
/* FDCAN 发送请求次数。 */
extern volatile uint32_t g_fdcan_tx_request_count;
/* FDCAN 发送成功次数。 */
extern volatile uint32_t g_fdcan_tx_ok_count;
/* FDCAN 发送失败次数。 */
extern volatile uint32_t g_fdcan_tx_fail_count;

#ifdef __cplusplus
}
#endif

#endif /* CAN_MOTOR_H */
