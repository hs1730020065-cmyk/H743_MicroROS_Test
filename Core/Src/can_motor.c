/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can_motor.c
  * @brief   Minimal FDCAN1 motor control frame sender.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "can_motor.h"
#include "fdcan.h"

#define CAN_MOTOR_CONTROL_ID          0x206U
#define CAN_MOTOR_CONTROL_DLC         FDCAN_DLC_BYTES_8

void CanMotor_Init(void)
{
  (void)HAL_FDCAN_Start(&hfdcan1);

  /*
   * Enable this later only after FDCAN1 RX FIFO0 is configured in CubeMX.
   *
   * (void)HAL_FDCAN_ActivateNotification(&hfdcan1,
   *                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
   *                                      0);
   */
}

HAL_StatusTypeDef CanMotor_SendRawSpeed(uint16_t raw_speed)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};
  uint8_t data[8] = {0};

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

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
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
