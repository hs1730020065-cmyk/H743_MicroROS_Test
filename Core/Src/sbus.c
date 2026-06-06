/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sbus.c
  * @brief   SBUS接收解析和手动控制量映射。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "sbus.h"
#include "usart.h"
#include <string.h>

/* DMA模式下直接接收一整帧SBUS数据。 */
static uint8_t sbus_rx_dma_buf[SBUS_FRAME_SIZE];
/* 中断模式下的一帧接收缓存。 */
static uint8_t sbus_rx_it_buf[SBUS_FRAME_SIZE];
/* 最近一次完整SBUS帧，供主循环或上层逻辑读取解析。 */
static uint8_t sbus_latest_frame[SBUS_FRAME_SIZE];
/* 标记当前是否成功启用了DMA接收。 */
static volatile uint8_t sbus_using_dma = 0U;
/* 已接收完整帧计数，可用于观察链路是否持续刷新。 */
static volatile uint32_t sbus_rx_frame_count = 0U;
/* 串口助手发送SBUS原始帧的测试模式开关。 */
static volatile uint8_t sbus_serial_assistant_test_enabled = 0U;
/* 串口助手测试模式下逐字节拼接SBUS帧的状态。 */
static uint8_t sbus_rx_stream_byte;
static uint8_t sbus_rx_stream_frame[SBUS_FRAME_SIZE];
static volatile uint8_t sbus_rx_stream_index = 0U;

#define UART5_TEXT_LINE_SIZE 32U

/* UART5文本命令测试模式：用一行ASCII命令模拟油门/转向输入。 */
static volatile uint8_t uart5_text_test_enabled = 0U;
static uint8_t uart5_text_rx_byte;
static char uart5_text_line[UART5_TEXT_LINE_SIZE];
static char uart5_text_ready_line[UART5_TEXT_LINE_SIZE];
static volatile uint8_t uart5_text_line_len = 0U;
static volatile uint8_t uart5_text_line_ready = 0U;

/* 将SBUS原始通道值映射到-1000..1000，并在中位附近设置死区。 */
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

/* 仅处理串口命令需要的ASCII大小写转换。 */
static char uart5_text_to_lower(char c)
{
  if ((c >= 'A') && (c <= 'Z'))
  {
    return (char)(c - 'A' + 'a');
  }

  return c;
}

/* 串口助手命令比较，大小写不敏感。 */
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

/* 发送调试文本，短超时避免长时间阻塞。 */
static void uart5_text_send(const char *text)
{
  (void)HAL_UART_Transmit(&huart5, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

void SBUS_Init(void)
{
  /* 真实SBUS接收初始化：清空测试模式状态并恢复UART5的SBUS配置。 */
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

  /* UART5中断始终打开，DMA失败时可退回普通中断接收。 */
  HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);

  /* 优先使用DMA整帧接收，降低CPU中断负担。 */
  if (huart5.hdmarx != NULL)
  {
    if (HAL_UART_Receive_DMA(&huart5, sbus_rx_dma_buf, SBUS_FRAME_SIZE) == HAL_OK)
    {
      sbus_using_dma = 1U;
      return;
    }
  }

  /* 无DMA或DMA启动失败时，使用中断方式一次接收25字节。 */
  (void)HAL_UART_Receive_IT(&huart5, sbus_rx_it_buf, SBUS_FRAME_SIZE);
}

void SBUS_TextCommandTest_Init(void)
{
  /* 文本命令测试会临时改成普通串口参数，便于串口助手直接输入命令。 */
  uart5_text_test_enabled = 0U;
  uart5_text_line_len = 0U;
  uart5_text_line_ready = 0U;

  (void)HAL_UART_Abort(&huart5);
  (void)HAL_UART_DeInit(&huart5);

  /*
   * UART5文本命令测试接线：
   * PC12 = UART5_TX -> USB-TTL RX
   * PB5  = UART5_RX -> USB-TTL TX
   * GND需要共地。
   *
   * 该模式会临时关闭SBUS真实接收使用的100000 8E2反相串口配置，
   * 改为串口助手常用的115200 8N1、无反相。
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

  /* 从这里开始，接收回调会进入文本命令分支。 */
  uart5_text_test_enabled = 1U;

  uart5_text_send("\r\nUART5 CMD TEST READY\r\n");
  uart5_text_send("Send: forward, reverse, stop, left, right\r\n");

  (void)HAL_UART_Receive_IT(&huart5, &uart5_text_rx_byte, 1U);
}

void SBUS_SerialAssistantTest_Init(void)
{
  /* 串口助手SBUS帧测试：普通串口收25字节载荷，验证解析链路。 */
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
   * 串口助手SBUS载荷测试接线：
   * PC12 = UART5_TX -> USB-TTL RX
   * PB5  = UART5_RX -> USB-TTL TX
   * GND需要共地。
   *
   * 为了USB-TTL和串口助手稳定调试，这里使用115200 8N1。
   * 发送内容仍然应是标准25字节SBUS帧；连接真实SBUS接收机时请使用SBUS_Init()，
   * 即100000 8E2反相配置。
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

  /* 从这里开始，接收回调会按SBUS帧头逐字节同步并组帧。 */
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

  /* 没有完整一行命令时，主循环无需处理。 */
  if (uart5_text_line_ready == 0U)
  {
    return false;
  }

  /* 接收回调会修改ready_line，因此复制时短暂关中断保护一致性。 */
  __disable_irq();
  (void)memcpy(line, uart5_text_ready_line, UART5_TEXT_LINE_SIZE);
  uart5_text_line_ready = 0U;
  __enable_irq();

  /* 每条命令先给控制量一个安全默认值，再按具体命令覆盖。 */
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

  /* 中文提示语的UTF-8字节，避免不同源码编码影响串口输出。 */
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

  /* 油门方向带阈值判断，避免小抖动频繁打印。 */
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

  /* 转向方向同样带阈值，靠近中位时视为回正。 */
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

  /* 方向未变化且距离上次发送不足500ms时不重复刷屏。 */
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
  /* DMA模式直接读取DMA缓存；中断/测试模式读取最近完整帧缓存。 */
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
    /* 帧头不匹配说明还没有对齐到一帧SBUS数据。 */
    out->frame_valid = 0U;
    return false;
  }

  /* SBUS的16个模拟通道按11位紧密打包在25字节帧中。 */
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

  /* 第23字节保存两个数字通道和链路状态标志。 */
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

  /* 当前约定：CH1控制转向，CH2控制油门。 */
  ctrl->steering = steering;
  ctrl->throttle = throttle;

  /* CH5作为手动模式开关，高于中位判定为开启。 */
  ctrl->manual_mode = (sbus->ch[4] > SBUS_MID_VALUE) ? 1U : 0U;

  ctrl->estop = 0U;

  /* CH6或SBUS failsafe触发急停。 */
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

      /* 文本命令以回车或换行结束，形成一条待处理命令。 */
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
        /* 兼容退格键和DEL键。 */
        if (uart5_text_line_len > 0U)
        {
          uart5_text_line_len--;
        }
      }
      else if ((c >= 0x20U) && (c <= 0x7EU))
      {
        /* 只收可打印ASCII字符，避免控制字符污染命令缓存。 */
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

      /* 等待SBUS帧头0x0F，实现串口字节流重新同步。 */
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
        /* 帧头已同步后继续收满25字节。 */
        sbus_rx_stream_frame[index] = c;
        index++;

        if (index >= SBUS_FRAME_SIZE)
        {
          /* 收到完整帧后发布到latest_frame，并重新等待下一帧帧头。 */
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

    /* 普通中断模式：一次回调代表25字节已收满，复制为最新帧。 */
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
    /* 中断接收出错后立即重新挂起接收，尽量恢复下一帧。 */
    (void)HAL_UART_Receive_IT(&huart5, sbus_rx_it_buf, SBUS_FRAME_SIZE);
  }
}

void UART5_IRQHandler(void)
{
  /* 将UART5硬件中断交给HAL统一分发到接收/错误回调。 */
  HAL_UART_IRQHandler(&huart5);
}
