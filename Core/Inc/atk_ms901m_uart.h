/**
 * @file        atk_ms901m_uart.h
 * @brief       ATK-MS901M 十轴传感器 USART2 通信接口声明
 */
#ifndef __ATK_MS901M_UART_H
#define __ATK_MS901M_UART_H

#include "main.h"

/* MS901M 串口 TX 引脚配置：USART2_TX -> PA2。 */
#define ATK_MS901M_UART_TX_GPIO_PORT            GPIOA
#define ATK_MS901M_UART_TX_GPIO_PIN             GPIO_PIN_2
#define ATK_MS901M_UART_TX_GPIO_AF              GPIO_AF7_USART2
#define ATK_MS901M_UART_TX_GPIO_CLK_ENABLE()    do { __HAL_RCC_GPIOA_CLK_ENABLE(); } while (0)

/* MS901M 串口 RX 引脚配置：USART2_RX -> PA3。 */
#define ATK_MS901M_UART_RX_GPIO_PORT            GPIOA
#define ATK_MS901M_UART_RX_GPIO_PIN             GPIO_PIN_3
#define ATK_MS901M_UART_RX_GPIO_AF              GPIO_AF7_USART2
#define ATK_MS901M_UART_RX_GPIO_CLK_ENABLE()    do { __HAL_RCC_GPIOA_CLK_ENABLE(); } while (0)

/* MS901M 使用的 UART 外设和中断号。 */
#define ATK_MS901M_UART_INTERFACE               USART2
#define ATK_MS901M_UART_IRQn                    USART2_IRQn
#define ATK_MS901M_UART_CLK_ENABLE()            do { __HAL_RCC_USART2_CLK_ENABLE(); } while (0)

/* 接收 FIFO 大小，单位：字节；用于缓存中断接收到的传感器数据。 */
#define ATK_MS901M_UART_RX_FIFO_BUF_SIZE        128U

/**
 * @brief       MS901M 串口底层接口函数
 * @note        上层 ATK-MS901M 驱动通过这些接口完成收发和 FIFO 管理。
 */
uint8_t atk_ms901m_uart_rx_fifo_write(uint8_t *dat, uint16_t len);
uint16_t atk_ms901m_uart_rx_fifo_read(uint8_t *dat, uint16_t len);
void atk_ms901m_rx_fifo_flush(void);
void atk_ms901m_uart_send(uint8_t *dat, uint8_t len);
void atk_ms901m_uart_init(uint32_t baudrate);
void ATK_MS901M_UART_IRQHandler(void);

#endif