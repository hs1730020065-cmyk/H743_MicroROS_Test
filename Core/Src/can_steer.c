/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can_steer.c
  * @brief   EPS 转向控制帧的最小发送实现。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "can_steer.h"
#include "can_motor.h"

/* EPS 控制命令帧 ID。 */
#define EPS_CMD_ID       0x314U
/* EPS 反馈帧 ID，当前仅保留占位，后续接收解析时使用。 */
#define EPS_FEEDBACK_ID  0x18FU

/* 发送 8 字节 EPS 控制帧，并同步更新 FDCAN 发送诊断计数。 */
static HAL_StatusTypeDef EpsSteer_SendFrame(uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef txHeader = {0};
  HAL_StatusTypeDef status;

  txHeader.Identifier = EPS_CMD_ID;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  /* 当前还未接收 EPS 反馈帧，先避免未使用宏告警。 */
  (void)EPS_FEEDBACK_ID;

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

HAL_StatusTypeDef EpsSteer_SendTargetAngle(int16_t angle_deg)
{
  uint8_t data[8] = {0};
  uint16_t raw = (uint16_t)angle_deg;

  /* 协议使用大端方式发送目标角度，data[0] 为角度控制命令字。 */
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
