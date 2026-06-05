/**
 ****************************************************************************************************
 * @file        atk_ms901m.h
 * @author      ALIENTEK
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS901M 姿态模块的对外接口。
 * @license     Copyright (c) 2020-2032, ALIENTEK
 ****************************************************************************************************
 * @attention
 *
 * 这个头文件只放模块协议编号、数据结构和函数声明。
 * 具体的串口收发、数据帧解析和单位换算在 atk_ms901m.c 里完成。
 *
 ****************************************************************************************************
 */

#ifndef __ATM_MS901M_H
#define __ATM_MS901M_H

#include "main.h"
#include "atk_ms901m_uart.h"

/* 模块一帧数据里，数据区最多 28 个字节。 */
#define ATK_MS901M_FRAME_DAT_MAX_SIZE       28

/* 模块主动发上来的数据类型。 */
#define ATK_MS901M_FRAME_ID_ATTITUDE        0x01    /* 欧拉角：roll、pitch、yaw。 */
#define ATK_MS901M_FRAME_ID_QUAT            0x02    /* 四元数：q0、q1、q2、q3。 */
#define ATK_MS901M_FRAME_ID_GYRO_ACCE       0x03    /* 陀螺仪和加速度计。 */
#define ATK_MS901M_FRAME_ID_MAG             0x04    /* 磁力计和温度。 */
#define ATK_MS901M_FRAME_ID_BARO            0x05    /* 气压、高度和温度。 */
#define ATK_MS901M_FRAME_ID_PORT            0x06    /* 模块 D0-D3 端口数据。 */

/* 读写模块参数时用的寄存器编号。 */
#define ATK_MS901M_FRAME_ID_REG_SAVE        0x00    /* 保存当前设置，掉电后仍然生效。 */
#define ATK_MS901M_FRAME_ID_REG_SENCAL      0x01    /* 启动传感器校准。 */
#define ATK_MS901M_FRAME_ID_REG_SENSTA      0x02    /* 读取传感器校准状态。 */
#define ATK_MS901M_FRAME_ID_REG_GYROFSR     0x03    /* 陀螺仪量程，影响 dps 换算。 */
#define ATK_MS901M_FRAME_ID_REG_ACCFSR      0x04    /* 加速度计量程，影响 G 值换算。 */
#define ATK_MS901M_FRAME_ID_REG_GYROBW      0x05    /* 陀螺仪滤波带宽。 */
#define ATK_MS901M_FRAME_ID_REG_ACCBW       0x06    /* 加速度计滤波带宽。 */
#define ATK_MS901M_FRAME_ID_REG_BAUD        0x07    /* 模块串口波特率。 */
#define ATK_MS901M_FRAME_ID_REG_RETURNSET   0x08    /* 选择模块主动上报哪些数据。 */
#define ATK_MS901M_FRAME_ID_REG_RETURNSET2  0x09    /* 选择模块主动上报的扩展数据。 */
#define ATK_MS901M_FRAME_ID_REG_RETURNRATE  0x0A    /* 模块主动上报的频率。 */
#define ATK_MS901M_FRAME_ID_REG_ALG         0x0B    /* 姿态解算算法相关设置。 */
#define ATK_MS901M_FRAME_ID_REG_ASM         0x0C    /* 模块安装方向设置。 */
#define ATK_MS901M_FRAME_ID_REG_GAUCAL      0x0D    /* 陀螺仪、加速度计校准相关设置。 */
#define ATK_MS901M_FRAME_ID_REG_BAUCAL      0x0E    /* 气压计校准相关设置。 */
#define ATK_MS901M_FRAME_ID_REG_LEDOFF      0x0F    /* 模块 LED 开关。 */
#define ATK_MS901M_FRAME_ID_REG_D0MODE      0x10    /* D0 端口工作模式。 */
#define ATK_MS901M_FRAME_ID_REG_D1MODE      0x11    /* D1 端口工作模式。 */
#define ATK_MS901M_FRAME_ID_REG_D2MODE      0x12    /* D2 端口工作模式。 */
#define ATK_MS901M_FRAME_ID_REG_D3MODE      0x13    /* D3 端口工作模式。 */
#define ATK_MS901M_FRAME_ID_REG_D1PULSE     0x16    /* D1 端口 PWM 高电平时间。 */
#define ATK_MS901M_FRAME_ID_REG_D3PULSE     0x1A    /* D3 端口 PWM 高电平时间。 */
#define ATK_MS901M_FRAME_ID_REG_D1PERIOD    0x1F    /* D1 端口 PWM 周期。 */
#define ATK_MS901M_FRAME_ID_REG_D3PERIOD    0x23    /* D3 端口 PWM 周期。 */
#define ATK_MS901M_FRAME_ID_REG_RESET       0x7F    /* 复位模块。 */

/* 区分“模块主动上传的数据帧”和“读写寄存器后的应答帧”。 */
#define ATK_MS901M_FRAME_ID_TYPE_UPLOAD     0       /* 主动上传帧。 */
#define ATK_MS901M_FRAME_ID_TYPE_ACK        1       /* 寄存器读写应答帧。 */

/* 姿态角，单位是度。 */
typedef struct
{
    float roll;                                     /* 横滚角，左右倾斜角。 */
    float pitch;                                    /* 俯仰角，抬头/低头角。 */
    float yaw;                                      /* 航向角，水平转向角。 */
} atk_ms901m_attitude_data_t;

/* 姿态四元数，适合给姿态融合或 ROS 姿态消息使用。 */
typedef struct
{
    float q0;                                       /* 四元数第 0 项。 */
    float q1;                                       /* 四元数第 1 项。 */
    float q2;                                       /* 四元数第 2 项。 */
    float q3;                                       /* 四元数第 3 项。 */
} atk_ms901m_quaternion_data_t;

/* 陀螺仪角速度数据。 */
typedef struct
{
    struct
    {
        int16_t x;                                  /* X 轴原始 ADC/协议值。 */
        int16_t y;                                  /* Y 轴原始 ADC/协议值。 */
        int16_t z;                                  /* Z 轴原始 ADC/协议值。 */
    } raw;
    float x;                                        /* X 轴角速度，单位 dps。 */
    float y;                                        /* Y 轴角速度，单位 dps。 */
    float z;                                        /* Z 轴角速度，单位 dps。 */
} atk_ms901m_gyro_data_t;

/* 加速度计数据。 */
typedef struct
{
    struct
    {
        int16_t x;                                  /* X 轴原始 ADC/协议值。 */
        int16_t y;                                  /* Y 轴原始 ADC/协议值。 */
        int16_t z;                                  /* Z 轴原始 ADC/协议值。 */
    } raw;
    float x;                                        /* X 轴加速度，单位 G。 */
    float y;                                        /* Y 轴加速度，单位 G。 */
    float z;                                        /* Z 轴加速度，单位 G。 */
} atk_ms901m_accelerometer_data_t;

/* 磁力计数据。 */
typedef struct
{
    int16_t x;                                      /* X 轴磁场原始值。 */
    int16_t y;                                      /* Y 轴磁场原始值。 */
    int16_t z;                                      /* Z 轴磁场原始值。 */
    float temperature;                              /* 模块温度，单位摄氏度。 */
} atk_ms901m_magnetometer_data_t;

/* 气压计数据。 */
typedef struct
{
    int32_t pressure;                               /* 气压，单位 Pa。 */
    int32_t altitude;                               /* 根据气压估算的高度，单位 cm。 */
    float temperature;                              /* 模块温度，单位摄氏度。 */
} atk_ms901m_barometer_data_t;

/* 模块 D0-D3 端口采样值。 */
typedef struct
{
    uint16_t d0;                                    /* D0 端口数据。 */
    uint16_t d1;                                    /* D1 端口数据。 */
    uint16_t d2;                                    /* D2 端口数据。 */
    uint16_t d3;                                    /* D3 端口数据。 */
} atk_ms901m_port_data_t;

/* 模块 LED 状态。 */
typedef enum
{
    ATK_MS901M_LED_STATE_ON  = 0x00,                /* LED 亮。 */
    ATK_MS901M_LED_STATE_OFF = 0x01,                /* LED 灭。 */
} atk_ms901m_led_state_t;

/* 模块端口编号。 */
typedef enum
{
    ATK_MS901M_PORT_D0 = 0x00,                      /* D0 端口。 */
    ATK_MS901M_PORT_D1 = 0x01,                      /* D1 端口。 */
    ATK_MS901M_PORT_D2 = 0x02,                      /* D2 端口。 */
    ATK_MS901M_PORT_D3 = 0x03,                      /* D3 端口。 */
} atk_ms901m_port_t;

/* D0-D3 端口可以配置成的工作模式。 */
typedef enum
{
    ATK_MS901M_PORT_MODE_ANALOG_INPUT   = 0x00,     /* 模拟输入。 */
    ATK_MS901M_PORT_MODE_INPUT          = 0x01,     /* 数字输入。 */
    ATK_MS901M_PORT_MODE_OUTPUT_HIGH    = 0x02,     /* 数字输出，高电平。 */
    ATK_MS901M_PORT_MODE_OUTPUT_LOW     = 0x03,     /* 数字输出，低电平。 */
    ATK_MS901M_PORT_MODE_OUTPUT_PWM     = 0x04,     /* PWM 输出。 */
} atk_ms901m_port_mode_t;

/* 本驱动函数的返回值。 */
#define ATK_MS901M_EOK      0                       /* 成功。 */
#define ATK_MS901M_ERROR    1                       /* 失败。 */
#define ATK_MS901M_EINVAL   2                       /* 参数不对。 */
#define ATK_MS901M_ETIMEOUT 3                       /* 等待模块返回超时。 */

/* 按寄存器编号读写模块参数。 */
uint8_t atk_ms901m_read_reg_by_id(uint8_t id, uint8_t *dat, uint32_t timeout);                                                                      /* 读一个模块寄存器。 */
uint8_t atk_ms901m_write_reg_by_id(uint8_t id, uint8_t len, uint8_t *dat);                                                                          /* 写一个模块寄存器。 */
uint8_t atk_ms901m_init(uint32_t baudrate);                                                                                                         /* 初始化串口，并读取陀螺仪/加速度计量程。 */

/* 读取模块主动上报的传感器数据。 */
uint8_t atk_ms901m_get_attitude(atk_ms901m_attitude_data_t *attitude_dat, uint32_t timeout);                                                        /* 读取 roll、pitch、yaw。 */
uint8_t atk_ms901m_get_quaternion(atk_ms901m_quaternion_data_t *quaternion_dat, uint32_t timeout);                                                  /* 读取四元数。 */
uint8_t atk_ms901m_get_gyro_accelerometer(atk_ms901m_gyro_data_t *gyro_dat, atk_ms901m_accelerometer_data_t *accelerometer_dat, uint32_t timeout);  /* 读取角速度和加速度。 */
uint8_t atk_ms901m_get_magnetometer(atk_ms901m_magnetometer_data_t *magnetometer_dat, uint32_t timeout);                                            /* 读取磁力计和温度。 */
uint8_t atk_ms901m_get_barometer(atk_ms901m_barometer_data_t *barometer_dat, uint32_t timeout);                                                     /* 读取气压、高度和温度。 */
uint8_t atk_ms901m_get_port(atk_ms901m_port_data_t *port_dat, uint32_t timeout);                                                                    /* 读取 D0-D3 端口数据。 */

/* 配置模块 LED 和 D0-D3 端口。 */
uint8_t atk_ms901m_get_led_state(atk_ms901m_led_state_t *state, uint32_t timeout);                                                                  /* 读取 LED 当前状态。 */
uint8_t atk_ms901m_set_led_state(atk_ms901m_led_state_t state, uint32_t timeout);                                                                   /* 设置 LED 亮灭。 */
uint8_t atk_ms901m_get_port_mode(atk_ms901m_port_t port, atk_ms901m_port_mode_t *mode, uint32_t timeout);                                           /* 读取指定端口模式。 */
uint8_t atk_ms901m_set_port_mode(atk_ms901m_port_t port, atk_ms901m_port_mode_t mode, uint32_t timeout);                                            /* 设置指定端口模式。 */
uint8_t atk_ms901m_get_port_pwm_pulse(atk_ms901m_port_t port, uint16_t *pulse, uint32_t timeout);                                                   /* 读取 D1/D3 的 PWM 高电平时间。 */
uint8_t atk_ms901m_set_port_pwm_pulse(atk_ms901m_port_t port, uint16_t pulse, uint32_t timeout);                                                    /* 设置 D1/D3 的 PWM 高电平时间。 */
uint8_t atk_ms901m_get_port_pwm_period(atk_ms901m_port_t port, uint16_t *period, uint32_t timeout);                                                 /* 读取 D1/D3 的 PWM 周期。 */
uint8_t atk_ms901m_set_port_pwm_period(atk_ms901m_port_t port, uint16_t period, uint32_t timeout);                                                  /* 设置 D1/D3 的 PWM 周期。 */

#endif