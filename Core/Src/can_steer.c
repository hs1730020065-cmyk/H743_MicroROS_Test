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

/*
 * EPS 执行器角度 -> 车轮转角的减速比。
 * 实测数据：车轮 10°=EPS 122, 20°=237, 30°=345，比值约 12.0。
 */
#define EPS_STEERING_RATIO          12.0f

/* EPS 角度单位：1°/bit（命令帧和反馈帧一致）。 */
#define EPS_FEEDBACK_ANGLE_SCALE    1.0f

/* 全局反馈状态变量。 */
volatile float g_eps_actual_angle_deg = 0.0f;
volatile uint32_t g_eps_feedback_rx_count = 0U;
volatile uint32_t g_eps_feedback_last_tick = 0U;

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

HAL_StatusTypeDef EpsSteer_SendTargetAngle(int16_t road_wheel_deg)
{
  uint8_t data[8] = {0};
  float eps_float;
  int16_t eps_raw;
  uint16_t raw;

  /* 车轮转角 -> EPS 执行器角度。 */
  eps_float = (float)road_wheel_deg * EPS_STEERING_RATIO;
  eps_raw = (int16_t)(eps_float + (eps_float >= 0.0f ? 0.5f : -0.5f));
  raw = (uint16_t)eps_raw;

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

/**
 * @brief       从 EPS 反馈帧 (0x18F) 中解析实际车轮转角
 * @param       data: 8 字节 CAN 数据载荷
 * @retval      实际车轮转角，单位：度（deg）
 *              正值 = 右转（EPS 协议方向）
 *              负值 = 左转（EPS 协议方向）
 * @note        data[1:2] 为大端 int16，单位 1°/bit（EPS 执行器角度），
 *              除以 EPS_STEERING_RATIO 换算为车轮转角。
 */
float EpsSteer_ParseFeedback(const uint8_t data[8])
{
  int16_t eps_raw;
  float eps_angle_deg;
  float road_wheel_deg;

  if (data == NULL)
  {
    return 0.0f;
  }

  /* 大端 int16: data[1] 为高字节，data[2] 为低字节，1°/bit。 */
  eps_raw = (int16_t)(((uint16_t)data[1] << 8U) | (uint16_t)data[2]);
  eps_angle_deg = (float)eps_raw * EPS_FEEDBACK_ANGLE_SCALE;

  /* EPS 执行器角度 -> 车轮转角。 */
  road_wheel_deg = eps_angle_deg / EPS_STEERING_RATIO;

  return road_wheel_deg;
}
