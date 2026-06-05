/**
 * @file        atk_ms901m_uart.c
 * @brief       ATK-MS901M 串口驱动适配层
 * @note        负责 USART2 初始化、发送、接收中断和 FIFO 缓存。
 */
#include "atk_ms901m_uart.h"
#include "usart.h"

/* MS901M 使用 USART2，对应 CubeMX 生成的 huart2。 */
static UART_HandleTypeDef *g_uart_handle = &huart2;

/* 串口接收环形 FIFO：中断写入，解析函数读取。 */
static struct
{
    uint8_t buf[ATK_MS901M_UART_RX_FIFO_BUF_SIZE];
    uint16_t size;
    uint16_t reader;
    uint16_t writer;
} g_uart_rx_fifo;

/**
 * @brief       向 MS901M 串口接收 FIFO 写入数据
 * @param       dat: 数据指针
 *              len: 数据长度，单位：字节
 * @retval      0: 写入成功
 *              1: 参数错误或 FIFO 尚未初始化
 * @note        主要在 USART2 接收中断中调用。如果上层长时间不读取，新数据可能覆盖旧数据。
 */
uint8_t atk_ms901m_uart_rx_fifo_write(uint8_t *dat, uint16_t len)
{
    uint16_t i;

    if ((dat == NULL) || (g_uart_rx_fifo.size == 0U))
    {
        return 1U;
    }

    for (i = 0U; i < len; i++)
    {
        g_uart_rx_fifo.buf[g_uart_rx_fifo.writer] = dat[i];
        g_uart_rx_fifo.writer = (uint16_t)((g_uart_rx_fifo.writer + 1U) % g_uart_rx_fifo.size);
    }

    return 0U;
}

/**
 * @brief       从 MS901M 串口接收 FIFO 读取数据
 * @param       dat: 数据缓冲区
 *              len: 期望读取的长度，单位：字节
 * @retval      实际读取到的长度，单位：字节
 * @note        如果 FIFO 中数据少于 len，则只返回当前已有的数据。
 */
uint16_t atk_ms901m_uart_rx_fifo_read(uint8_t *dat, uint16_t len)
{
    uint16_t fifo_usage;
    uint16_t i;

    if ((dat == NULL) || (g_uart_rx_fifo.size == 0U))
    {
        return 0U;
    }

    if (g_uart_rx_fifo.writer >= g_uart_rx_fifo.reader)
    {
        fifo_usage = (uint16_t)(g_uart_rx_fifo.writer - g_uart_rx_fifo.reader);
    }
    else
    {
        fifo_usage = (uint16_t)(g_uart_rx_fifo.size - g_uart_rx_fifo.reader + g_uart_rx_fifo.writer);
    }

    if (len > fifo_usage)
    {
        len = fifo_usage;
    }

    for (i = 0U; i < len; i++)
    {
        dat[i] = g_uart_rx_fifo.buf[g_uart_rx_fifo.reader];
        g_uart_rx_fifo.reader = (uint16_t)((g_uart_rx_fifo.reader + 1U) % g_uart_rx_fifo.size);
    }

    return len;
}

/**
 * @brief       清空 MS901M 串口接收 FIFO
 * @param       无
 * @retval      无
 * @note        通过移动写指针实现清空，用于丢弃旧数据。
 */
void atk_ms901m_rx_fifo_flush(void)
{
    g_uart_rx_fifo.writer = g_uart_rx_fifo.reader;
}

/**
 * @brief       通过串口向 MS901M 发送命令数据
 * @param       dat: 待发送的数据缓冲区
 *              len: 待发送的长度，单位：字节
 * @retval      无
 * @note        使用阻塞发送方式，适合初始化和配置短命令。
 */
void atk_ms901m_uart_send(uint8_t *dat, uint8_t len)
{
    (void)HAL_UART_Transmit(g_uart_handle, dat, len, HAL_MAX_DELAY);
}

/**
 * @brief       初始化 MS901M 使用的 USART2 串口和接收 FIFO
 * @param       baudrate: 串口波特率，例如 115200
 * @retval      无
 * @note        初始化后开启 RXNE 接收中断，每收到 1 字节都写入 FIFO。
 */
void atk_ms901m_uart_init(uint32_t baudrate)
{
    g_uart_handle = &huart2;
    g_uart_rx_fifo.size = ATK_MS901M_UART_RX_FIFO_BUF_SIZE;
    g_uart_rx_fifo.reader = 0U;
    g_uart_rx_fifo.writer = 0U;

    g_uart_handle->Instance = ATK_MS901M_UART_INTERFACE;
    g_uart_handle->Init.BaudRate = baudrate;
    g_uart_handle->Init.WordLength = UART_WORDLENGTH_8B;
    g_uart_handle->Init.StopBits = UART_STOPBITS_1;
    g_uart_handle->Init.Parity = UART_PARITY_NONE;
    g_uart_handle->Init.Mode = UART_MODE_TX_RX;
    g_uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart_handle->Init.OverSampling = UART_OVERSAMPLING_16;
    g_uart_handle->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    g_uart_handle->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    g_uart_handle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(g_uart_handle) != HAL_OK)
    {
        return;
    }
    if (HAL_UARTEx_SetTxFifoThreshold(g_uart_handle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        return;
    }
    if (HAL_UARTEx_SetRxFifoThreshold(g_uart_handle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        return;
    }
    if (HAL_UARTEx_DisableFifoMode(g_uart_handle) != HAL_OK)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(g_uart_handle);
    __HAL_UART_ENABLE_IT(g_uart_handle, UART_IT_RXNE);
}

/**
 * @brief       MS901M 串口中断处理函数
 * @param       无
 * @retval      无
 * @note        清除溢出错误，并把收到的单字节数据写入 FIFO。
 */
void ATK_MS901M_UART_IRQHandler(void)
{
    uint8_t tmp;

    if ((g_uart_handle == NULL) || (g_uart_handle->Instance == NULL) || (g_uart_rx_fifo.size == 0U))
    {
        return;
    }

    if (__HAL_UART_GET_FLAG(g_uart_handle, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(g_uart_handle);
        (void)g_uart_handle->Instance->ISR;
        (void)g_uart_handle->Instance->RDR;
    }

    if (__HAL_UART_GET_FLAG(g_uart_handle, UART_FLAG_RXNE) != RESET)
    {
        tmp = (uint8_t)(g_uart_handle->Instance->RDR & 0xFFU);
        (void)atk_ms901m_uart_rx_fifo_write(&tmp, 1U);
    }
}