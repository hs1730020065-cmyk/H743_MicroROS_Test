#ifndef __ATK_MS901M_UART_H
#define __ATK_MS901M_UART_H

#include "main.h"

#define ATK_MS901M_UART_TX_GPIO_PORT            GPIOA
#define ATK_MS901M_UART_TX_GPIO_PIN             GPIO_PIN_2
#define ATK_MS901M_UART_TX_GPIO_AF              GPIO_AF7_USART2
#define ATK_MS901M_UART_TX_GPIO_CLK_ENABLE()    do { __HAL_RCC_GPIOA_CLK_ENABLE(); } while (0)

#define ATK_MS901M_UART_RX_GPIO_PORT            GPIOA
#define ATK_MS901M_UART_RX_GPIO_PIN             GPIO_PIN_3
#define ATK_MS901M_UART_RX_GPIO_AF              GPIO_AF7_USART2
#define ATK_MS901M_UART_RX_GPIO_CLK_ENABLE()    do { __HAL_RCC_GPIOA_CLK_ENABLE(); } while (0)

#define ATK_MS901M_UART_INTERFACE               USART2
#define ATK_MS901M_UART_IRQn                    USART2_IRQn
#define ATK_MS901M_UART_CLK_ENABLE()            do { __HAL_RCC_USART2_CLK_ENABLE(); } while (0)

#define ATK_MS901M_UART_RX_FIFO_BUF_SIZE        128U

uint8_t atk_ms901m_uart_rx_fifo_write(uint8_t *dat, uint16_t len);
uint16_t atk_ms901m_uart_rx_fifo_read(uint8_t *dat, uint16_t len);
void atk_ms901m_rx_fifo_flush(void);
void atk_ms901m_uart_send(uint8_t *dat, uint8_t len);
void atk_ms901m_uart_init(uint32_t baudrate);
void ATK_MS901M_UART_IRQHandler(void);

#endif