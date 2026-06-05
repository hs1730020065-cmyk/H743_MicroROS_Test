/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can_motor.c
  * @brief   FDCAN1 电机控制帧的最小发送实现。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "can_motor.h"
#include "fdcan.h"

#define CAN_MOTOR_CONTROL_ID          0x206U
#define CAN_MOTOR_CONTROL_DLC         FDCAN_DLC_BYTES_8

volatile HAL_StatusTypeDef g_fdcan_start_status = HAL_ERROR;
volatile uint32_t g_fdcan_hal_error = 0U;
volatile uint32_t g_fdcan_tx_fifo_free_level = 0U;
volatile uint32_t g_fdcan_last_error_code = 0U;
volatile uint32_t g_fdcan_activity = 0U;
volatile uint32_t g_fdcan_error_passive = 0U;
volatile uint32_t g_fdcan_warning = 0U;
volatile uint32_t g_fdcan_bus_off = 0U;
volatile uint32_t g_fdcan_tx_error_count = 0U;
volatile uint32_t g_fdcan_rx_error_count = 0U;
volatile uint32_t g_fdcan_tx_request_count = 0U;
volatile uint32_t g_fdcan_tx_ok_count = 0U;
volatile uint32_t g_fdcan_tx_fail_count = 0U;

void CanMotor_Init(void)
{
  g_fdcan_start_status = HAL_FDCAN_Start(&hfdcan1);
  CanMotor_UpdateDiagnostics();

  /*
   * 只有在 CubeMX 中配置好 FDCAN1 RX FIFO0 后，后续再启用这里的接收通知。
   *
   * (void)HAL_FDCAN_ActivateNotification(&hfdcan1,
   *                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
   *                                      0);
   */
}

void CanMotor_UpdateDiagnostics(void)
{
  FDCAN_ProtocolStatusTypeDef protocol_status = {0};
  FDCAN_ErrorCountersTypeDef error_counters = {0};

  g_fdcan_hal_error = HAL_FDCAN_GetError(&hfdcan1);
  g_fdcan_tx_fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);

  if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) == HAL_OK)
  {
    g_fdcan_last_error_code = protocol_status.LastErrorCode;
    g_fdcan_activity = protocol_status.Activity;
    g_fdcan_error_passive = protocol_status.ErrorPassive;
    g_fdcan_warning = protocol_status.Warning;
    g_fdcan_bus_off = protocol_status.BusOff;
  }

  if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters) == HAL_OK)
  {
    g_fdcan_tx_error_count = error_counters.TxErrorCnt;
    g_fdcan_rx_error_count = error_counters.RxErrorCnt;
  }
}

HAL_StatusTypeDef CanMotor_SendRawSpeed(uint16_t raw_speed)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};
  uint8_t data[8] = {0};
  HAL_StatusTypeDef status;

  txHeader.Identifier = CAN_MOTOR_CONTROL_ID;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = CAN_MOTOR_CONTROL_DLC;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  data[0] = 0x07U;
  data[1] = 0x00U;
  data[2] = 0x00U;
  data[3] = (uint8_t)(raw_speed & 0x00FFU);
  data[4] = (uint8_t)((raw_speed >> 8U) & 0x00FFU);
  data[5] = 0x00U;
  data[6] = 0x00U;
  data[7] = 0x00U;

  status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
  g_fdcan_tx_request_count++;

  if (status == HAL_OK)
  {
    g_fdcan_tx_ok_count++;
  }
  else
  {
    g_fdcan_tx_fail_count++;
  }

  CanMotor_UpdateDiagnostics();

  return status;
}

HAL_StatusTypeDef CanMotor_Stop(void)
{
  return CanMotor_SendRawSpeed(0x0000U);
}

HAL_StatusTypeDef CanMotor_Forward(uint16_t speed_abs)
{
  uint16_t raw_speed;

  if (speed_abs == 0U)
  {
    raw_speed = 0x0000U;
  }
  else
  {
    raw_speed = (uint16_t)(0x10000UL - (uint32_t)speed_abs);
  }

  return CanMotor_SendRawSpeed(raw_speed);
}

HAL_StatusTypeDef CanMotor_Reverse(uint16_t speed_abs)
{
  return CanMotor_SendRawSpeed(speed_abs);
}
