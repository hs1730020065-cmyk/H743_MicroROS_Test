/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sbus.c
  * @brief   SBUS receiver parsing and manual control mapping.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "sbus.h"
#include "usart.h"
#include <string.h>

static uint8_t sbus_rx_dma_buf[SBUS_FRAME_SIZE];
static uint8_t sbus_rx_it_buf[SBUS_FRAME_SIZE];
static uint8_t sbus_latest_frame[SBUS_FRAME_SIZE];
static volatile uint8_t sbus_using_dma = 0U;
static volatile uint32_t sbus_rx_frame_count = 0U;
static volatile uint8_t sbus_serial_assistant_test_enabled = 0U;
static uint8_t sbus_rx_stream_byte;
static uint8_t sbus_rx_stream_frame[SBUS_FRAME_SIZE];
static volatile uint8_t sbus_rx_stream_index = 0U;

#define UART5_TEXT_LINE_SIZE 32U

static volatile uint8_t uart5_text_test_enabled = 0U;
static uint8_t uart5_text_rx_byte;
static char uart5_text_line[UART5_TEXT_LINE_SIZE];
static char uart5_text_ready_line[UART5_TEXT_LINE_SIZE];
static volatile uint8_t uart5_text_line_len = 0U;
static volatile uint8_t uart5_text_line_ready = 0U;

static int16_t sbus_map_channel_to_1000(uint16_t value)
{
  int32_t v = (int32_t)value - SBUS_MID_VALUE;

  if ((v > -SBUS_DEADZONE) && (v < SBUS_DEADZONE))
  {
    return 0;
  }

  if (v > 0)
  {
    v = (v * 1000) / (SBUS_MAX_VALUE - SBUS_MID_VALUE);
  }
  else
  {
    v = (v * 1000) / (SBUS_MID_VALUE - SBUS_MIN_VALUE);
  }

  if (v > 1000)
  {
    v = 1000;
  }
  if (v < -1000)
  {
    v = -1000;
  }

  return (int16_t)v;
}

static char uart5_text_to_lower(char c)
{
  if ((c >= 'A') && (c <= 'Z'))
  {
    return (char)(c - 'A' + 'a');
  }

  return c;
}

static bool uart5_text_cmd_equal(const char *line, const char *cmd)
{
  while ((*line != '\0') && (*cmd != '\0'))
  {
    if (uart5_text_to_lower(*line) != uart5_text_to_lower(*cmd))
    {
      return false;
    }

    line++;
    cmd++;
  }

  return ((*line == '\0') && (*cmd == '\0'));
}

static void uart5_text_send(const char *text)
{
  (void)HAL_UART_Transmit(&huart5, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

void SBUS_Init(void)
{
  uart5_text_test_enabled = 0U;
  uart5_text_line_len = 0U;
  uart5_text_line_ready = 0U;
  sbus_using_dma = 0U;
  sbus_serial_assistant_test_enabled = 0U;
  sbus_rx_frame_count = 0U;
  sbus_rx_stream_index = 0U;

  for (uint32_t i = 0U; i < SBUS_FRAME_SIZE; i++)
  {
    sbus_rx_dma_buf[i] = 0U;
    sbus_rx_it_buf[i] = 0U;
    sbus_latest_frame[i] = 0U;
    sbus_rx_stream_frame[i] = 0U;
  }

  (void)HAL_UART_Abort(&huart5);
  (void)HAL_UART_DeInit(&huart5);
  MX_UART5_Init();

  HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);

  if (huart5.hdmarx != NULL)
  {
    if (HAL_UART_Receive_DMA(&huart5, sbus_rx_dma_buf, SBUS_FRAME_SIZE) == HAL_OK)
    {
      sbus_using_dma = 1U;
      return;
    }
  }

  (void)HAL_UART_Receive_IT(&huart5, sbus_rx_it_buf, SBUS_FRAME_SIZE);
}

void SBUS_TextCommandTest_Init(void)
{
  uart5_text_test_enabled = 0U;
  uart5_text_line_len = 0U;
  uart5_text_line_ready = 0U;

  (void)HAL_UART_Abort(&huart5);
  (void)HAL_UART_DeInit(&huart5);

  /*
   * UART5 text command test for serial assistant:
   * PC12 = UART5_TX -> USB-TTL RX
   * PB5  = UART5_RX -> USB-TTL TX
   * GND must be common.
   *
   * This temporarily disables the SBUS-style 100000 8E2 inverted UART setting
   * and uses normal serial assistant parameters: 115200 8N1, no inversion.
   */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);

  uart5_text_test_enabled = 1U;

  uart5_text_send("\r\nUART5 CMD TEST READY\r\n");
  uart5_text_send("Send: forward, reverse, stop, left, right\r\n");

  (void)HAL_UART_Receive_IT(&huart5, &uart5_text_rx_byte, 1U);
}

void SBUS_SerialAssistantTest_Init(void)
{
  uart5_text_test_enabled = 0U;
  uart5_text_line_len = 0U;
  uart5_text_line_ready = 0U;
  sbus_using_dma = 0U;
  sbus_rx_frame_count = 0U;
  sbus_rx_stream_index = 0U;

  for (uint32_t i = 0U; i < SBUS_FRAME_SIZE; i++)
  {
    sbus_latest_frame[i] = 0U;
    sbus_rx_it_buf[i] = 0U;
    sbus_rx_stream_frame[i] = 0U;
  }

  (void)HAL_UART_Abort(&huart5);
  (void)HAL_UART_DeInit(&huart5);

  /*
   * Serial assistant SBUS payload test:
   * PC12 = UART5_TX -> USB-TTL RX
   * PB5  = UART5_RX -> USB-TTL TX
   * GND must be common.
   *
   * Use 115200 baud, 8 data bits, no parity, 1 stop bit for stable USB-TTL
   * serial-assistant verification. The transmitted payload is still a standard
   * 25-byte SBUS frame. Use SBUS_Init() instead for a real inverted SBUS
   * receiver at 100000 8E2.
   */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);

  sbus_serial_assistant_test_enabled = 1U;
  SBUS_SendSerialAssistantReadyText();

  (void)HAL_UART_Receive_IT(&huart5, &sbus_rx_stream_byte, 1U);
}

uint32_t SBUS_GetRxFrameCount(void)
{
  return sbus_rx_frame_count;
}

void SBUS_SendSerialAssistantReadyText(void)
{
  static const uint8_t msg[] = {'R', 'E', 'A', 'D', 'Y', '\r', '\n'};

  (void)HAL_UART_Transmit(&huart5, (uint8_t *)msg, (uint16_t)sizeof(msg), 100U);
}

void SBUS_SendSerialAssistantBadFrameText(void)
{
  static const uint8_t msg[] = {'B', 'A', 'D', ' ', 'F', 'R', 'A', 'M', 'E', '\r', '\n'};

  (void)HAL_UART_Transmit(&huart5, (uint8_t *)msg, (uint16_t)sizeof(msg), 100U);
}

bool SBUS_TextCommandTest_Process(SbusControl_t *ctrl)
{
  char line[UART5_TEXT_LINE_SIZE];
  bool handled = false;

  if (uart5_text_line_ready == 0U)
  {
    return false;
  }

  __disable_irq();
  (void)memcpy(line, uart5_text_ready_line, UART5_TEXT_LINE_SIZE);
  uart5_text_line_ready = 0U;
  __enable_irq();

  if (ctrl != NULL)
  {
    ctrl->throttle = 0;
    ctrl->steering = 0;
    ctrl->manual_mode = 1U;
    ctrl->estop = 0U;
  }

  if (uart5_text_cmd_equal(line, "forward") || uart5_text_cmd_equal(line, "f"))
  {
    if (ctrl != NULL)
    {
      ctrl->throttle = 500;
    }
    uart5_text_send("OK FORWARD\r\n");
    handled = true;
  }
  else if (uart5_text_cmd_equal(line, "reverse") || uart5_text_cmd_equal(line, "back") || uart5_text_cmd_equal(line, "r"))
  {
    if (ctrl != NULL)
    {
      ctrl->throttle = -500;
    }
    uart5_text_send("OK REVERSE\r\n");
    handled = true;
  }
  else if (uart5_text_cmd_equal(line, "stop") || uart5_text_cmd_equal(line, "s"))
  {
    uart5_text_send("OK STOP\r\n");
    handled = true;
  }
  else if (uart5_text_cmd_equal(line, "left") || uart5_text_cmd_equal(line, "l"))
  {
    if (ctrl != NULL)
    {
      ctrl->steering = -500;
    }
    uart5_text_send("OK LEFT\r\n");
    handled = true;
  }
  else if (uart5_text_cmd_equal(line, "right"))
  {
    if (ctrl != NULL)
    {
      ctrl->steering = 500;
    }
    uart5_text_send("OK RIGHT\r\n");
    handled = true;
  }
  else
  {
    uart5_text_send("ERR UNKNOWN CMD\r\n");
  }

  return handled;
}

void SBUS_SendControlDirectionText(const SbusControl_t *ctrl)
{
  static int8_t last_throttle_direction = 127;
  static int8_t last_steering_direction = 127;
  static uint32_t last_report_tick = 0U;
  int8_t throttle_direction = 0;
  int8_t steering_direction = 0;
  uint32_t now = HAL_GetTick();
  const uint8_t *throttle_msg = NULL;
  const uint8_t *steering_msg = NULL;
  uint16_t throttle_msg_len = 0U;
  uint16_t steering_msg_len = 0U;

  static const uint8_t msg_forward[] = {0xE5, 0x89, 0x8D, 0xE8, 0xBF, 0x9B, '\r', '\n'};
  static const uint8_t msg_reverse[] = {0xE5, 0x90, 0x8E, 0xE9, 0x80, 0x80, '\r', '\n'};
  static const uint8_t msg_stop[] = {0xE5, 0x81, 0x9C, 0xE6, 0xAD, 0xA2, '\r', '\n'};
  static const uint8_t msg_left[] = {0xE5, 0xB7, 0xA6, 0xE8, 0xBD, 0xAC, '\r', '\n'};
  static const uint8_t msg_right[] = {0xE5, 0x8F, 0xB3, 0xE8, 0xBD, 0xAC, '\r', '\n'};
  static const uint8_t msg_center[] = {0xE5, 0x9B, 0x9E, 0xE6, 0xAD, 0xA3, '\r', '\n'};
  static const uint8_t msg_space[] = {' '};
  static const uint8_t msg_newline[] = {'\r', '\n'};

  if (ctrl == NULL)
  {
    return;
  }

  if (ctrl->throttle > 100)
  {
    throttle_direction = 1;
    throttle_msg = msg_forward;
    throttle_msg_len = (uint16_t)(sizeof(msg_forward) - 2U);
  }
  else if (ctrl->throttle < -100)
  {
    throttle_direction = -1;
    throttle_msg = msg_reverse;
    throttle_msg_len = (uint16_t)(sizeof(msg_reverse) - 2U);
  }
  else
  {
    throttle_direction = 0;
    throttle_msg = msg_stop;
    throttle_msg_len = (uint16_t)(sizeof(msg_stop) - 2U);
  }

  if (ctrl->steering < -100)
  {
    steering_direction = -1;
    steering_msg = msg_left;
    steering_msg_len = (uint16_t)(sizeof(msg_left) - 2U);
  }
  else if (ctrl->steering > 100)
  {
    steering_direction = 1;
    steering_msg = msg_right;
    steering_msg_len = (uint16_t)(sizeof(msg_right) - 2U);
  }
  else
  {
    steering_direction = 0;
    steering_msg = msg_center;
    steering_msg_len = (uint16_t)(sizeof(msg_center) - 2U);
  }

  if ((throttle_direction == last_throttle_direction) &&
      (steering_direction == last_steering_direction) &&
      ((now - last_report_tick) < 500U))
  {
    return;
  }

  last_throttle_direction = throttle_direction;
  last_steering_direction = steering_direction;
  last_report_tick = now;

  (void)HAL_UART_Transmit(&huart5, (uint8_t *)throttle_msg, throttle_msg_len, 100U);
  (void)HAL_UART_Transmit(&huart5, (uint8_t *)msg_space, (uint16_t)sizeof(msg_space), 100U);
  (void)HAL_UART_Transmit(&huart5, (uint8_t *)steering_msg, steering_msg_len, 100U);
  (void)HAL_UART_Transmit(&huart5, (uint8_t *)msg_newline, (uint16_t)sizeof(msg_newline), 100U);
}

const uint8_t *SBUS_GetRxBuffer(void)
{
  if (sbus_using_dma)
  {
    return sbus_rx_dma_buf;
  }

  return sbus_latest_frame;
}

bool SBUS_TryParseFrame(const uint8_t frame[SBUS_FRAME_SIZE], SbusData_t *out)
{
  if ((frame == NULL) || (out == NULL))
  {
    return false;
  }

  if (frame[0] != SBUS_HEADER)
  {
    out->frame_valid = 0U;
    return false;
  }

  out->ch[0]  = ((frame[1]      | frame[2]  << 8) & 0x07FF);
  out->ch[1]  = ((frame[2] >> 3 | frame[3]  << 5) & 0x07FF);
  out->ch[2]  = ((frame[3] >> 6 | frame[4]  << 2 | frame[5] << 10) & 0x07FF);
  out->ch[3]  = ((frame[5] >> 1 | frame[6]  << 7) & 0x07FF);
  out->ch[4]  = ((frame[6] >> 4 | frame[7]  << 4) & 0x07FF);
  out->ch[5]  = ((frame[7] >> 7 | frame[8]  << 1 | frame[9] << 9) & 0x07FF);
  out->ch[6]  = ((frame[9] >> 2 | frame[10] << 6) & 0x07FF);
  out->ch[7]  = ((frame[10]>> 5 | frame[11] << 3) & 0x07FF);

  out->ch[8]  = ((frame[12]     | frame[13] << 8) & 0x07FF);
  out->ch[9]  = ((frame[13]>> 3 | frame[14] << 5) & 0x07FF);
  out->ch[10] = ((frame[14]>> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF);
  out->ch[11] = ((frame[16]>> 1 | frame[17] << 7) & 0x07FF);
  out->ch[12] = ((frame[17]>> 4 | frame[18] << 4) & 0x07FF);
  out->ch[13] = ((frame[18]>> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF);
  out->ch[14] = ((frame[20]>> 2 | frame[21] << 6) & 0x07FF);
  out->ch[15] = ((frame[21]>> 5 | frame[22] << 3) & 0x07FF);

  uint8_t flags = frame[23];

  out->ch17       = (flags >> 0) & 0x01U;
  out->ch18       = (flags >> 1) & 0x01U;
  out->frame_lost = (flags >> 2) & 0x01U;
  out->failsafe   = (flags >> 3) & 0x01U;

  out->frame_valid = 1U;

  return true;
}

void SBUS_ConvertToControl(const SbusData_t *sbus, SbusControl_t *ctrl)
{
  if ((sbus == NULL) || (ctrl == NULL))
  {
    return;
  }

  int16_t steering = sbus_map_channel_to_1000(sbus->ch[0]);
  int16_t throttle = sbus_map_channel_to_1000(sbus->ch[1]);

  ctrl->steering = steering;
  ctrl->throttle = throttle;

  ctrl->manual_mode = (sbus->ch[4] > SBUS_MID_VALUE) ? 1U : 0U;

  ctrl->estop = 0U;

  if (sbus->ch[5] > SBUS_MID_VALUE)
  {
    ctrl->estop = 1U;
  }

  if (sbus->failsafe)
  {
    ctrl->estop = 1U;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart5)
  {
    if (uart5_text_test_enabled != 0U)
    {
      uint8_t c = uart5_text_rx_byte;

      if ((c == '\r') || (c == '\n'))
      {
        if ((uart5_text_line_len > 0U) && (uart5_text_line_ready == 0U))
        {
          for (uint32_t i = 0U; i < uart5_text_line_len; i++)
          {
            uart5_text_ready_line[i] = uart5_text_line[i];
          }
          uart5_text_ready_line[uart5_text_line_len] = '\0';
          uart5_text_line_len = 0U;
          uart5_text_line_ready = 1U;
        }
      }
      else if ((c == 0x08U) || (c == 0x7FU))
      {
        if (uart5_text_line_len > 0U)
        {
          uart5_text_line_len--;
        }
      }
      else if ((c >= 0x20U) && (c <= 0x7EU))
      {
        if (uart5_text_line_len < (UART5_TEXT_LINE_SIZE - 1U))
        {
          uart5_text_line[uart5_text_line_len] = (char)c;
          uart5_text_line_len++;
        }
      }

      (void)HAL_UART_Receive_IT(&huart5, &uart5_text_rx_byte, 1U);
      return;
    }

    if (sbus_serial_assistant_test_enabled != 0U)
    {
      uint8_t c = sbus_rx_stream_byte;
      uint8_t index = sbus_rx_stream_index;

      if (index == 0U)
      {
        if (c == SBUS_HEADER)
        {
          sbus_rx_stream_frame[0] = c;
          sbus_rx_stream_index = 1U;
        }
      }
      else
      {
        sbus_rx_stream_frame[index] = c;
        index++;

        if (index >= SBUS_FRAME_SIZE)
        {
          for (uint32_t i = 0U; i < SBUS_FRAME_SIZE; i++)
          {
            sbus_latest_frame[i] = sbus_rx_stream_frame[i];
          }

          sbus_rx_frame_count++;
          sbus_rx_stream_index = 0U;
        }
        else
        {
          sbus_rx_stream_index = index;
        }
      }

      (void)HAL_UART_Receive_IT(&huart5, &sbus_rx_stream_byte, 1U);
      return;
    }

    for (uint32_t i = 0U; i < SBUS_FRAME_SIZE; i++)
    {
      sbus_latest_frame[i] = sbus_rx_it_buf[i];
    }

    sbus_rx_frame_count++;

    (void)HAL_UART_Receive_IT(&huart5, sbus_rx_it_buf, SBUS_FRAME_SIZE);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart == &huart5) && (sbus_using_dma == 0U))
  {
    (void)HAL_UART_Receive_IT(&huart5, sbus_rx_it_buf, SBUS_FRAME_SIZE);
  }
}

void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart5);
}
