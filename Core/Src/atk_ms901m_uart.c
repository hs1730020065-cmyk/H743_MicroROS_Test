#include "atk_ms901m_uart.h"
#include "usart.h"

static UART_HandleTypeDef *g_uart_handle = &huart2;

static struct
{
    uint8_t buf[ATK_MS901M_UART_RX_FIFO_BUF_SIZE];
    uint16_t size;
    uint16_t reader;
    uint16_t writer;
} g_uart_rx_fifo;

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

void atk_ms901m_rx_fifo_flush(void)
{
    g_uart_rx_fifo.writer = g_uart_rx_fifo.reader;
}

void atk_ms901m_uart_send(uint8_t *dat, uint8_t len)
{
    (void)HAL_UART_Transmit(g_uart_handle, dat, len, HAL_MAX_DELAY);
}

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