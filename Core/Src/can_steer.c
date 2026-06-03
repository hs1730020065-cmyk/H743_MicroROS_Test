/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can_steer.c
  * @brief   Minimal EPS steering control frame sender.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "can_steer.h"

#define EPS_CMD_ID       0x314U
#define EPS_FEEDBACK_ID  0x18FU

static HAL_StatusTypeDef EpsSteer_SendFrame(uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef txHeader = {0};

  txHeader.Identifier = EPS_CMD_ID;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  (void)EPS_FEEDBACK_ID;

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}

HAL_StatusTypeDef EpsSteer_SendTargetAngle(int16_t angle_deg)
{
  uint8_t data[8] = {0};
  uint16_t raw = (uint16_t)angle_deg;

  data[0] = 0x01U;
  data[1] = (uint8_t)((raw >> 8U) & 0xFFU);
  data[2] = (uint8_t)(raw & 0xFFU);

  return EpsSteer_SendFrame(data);
}

HAL_StatusTypeDef EpsSteer_SetManualMode(void)
{
  uint8_t data[8] = {0};

  return EpsSteer_SendFrame(data);
}

HAL_StatusTypeDef EpsSteer_SetZero(void)
{
  return EpsSteer_SendTargetAngle(0);
}
