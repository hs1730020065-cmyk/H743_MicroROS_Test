/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * 文件名            : freertos.c
  * 描述              : FreeRTOS任务、micro-ROS通信、底盘控制、IMU和SBUS采集调度
  ******************************************************************************
  * @attention
  *
  * 版权所有 (c) 2026 STMicroelectronics。
  * 保留所有权利。
  *
  * 本软件遵循LICENSE文件中的许可条款，
  * 该文件位于本软件组件的根目录。
  * 如果未随附LICENSE文件，则本软件按现状提供。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"       /* FreeRTOS 内核主头文件，提供任务调度、队列、信号量等基础 API */
#include "task.h"           /* FreeRTOS 任务管理 API：任务创建、删除、延时、优先级等 */
#include "main.h"           /* STM32CubeMX 生成的主头文件，包含所有外设句柄和系统配置 */
#include "FreeRTOS.h"       /* FreeRTOS 内核头文件（重复包含，由 CubeMX 模板生成） */
#include "cmsis_os2.h"      /* CMSIS-RTOS V2 封装层，将 FreeRTOS API 映射为 CMSIS 标准接口 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>        /* C 标准布尔类型：bool、true、false */
#include <math.h>           /* C 标准数学库：sinf、cosf、fabsf 等浮点运算 */
#include <stddef.h>         /* C 标准定义：NULL、size_t、offsetof 等 */
#include <stdint.h>         /* C 标准整数类型：uint8_t、uint32_t、int64_t 等固定宽度类型 */
#include <rcl/rcl.h>        /* ROS2 客户端库核心 API：节点、发布器、订阅器、定时器等 */
#include <rcl/error_handling.h> /* ROS2 错误处理：rcl_reset_error() 清除错误状态 */
#include <rclc/rclc.h>      /* micro-ROS 的 rclc 封装：简化嵌入式端的 ROS2 API */
#include <rclc/executor.h>  /* rclc executor：单线程事件循环，驱动订阅回调和定时器 */
#include <rclc/timer.h>     /* rclc 定时器：周期性触发回调函数 */
#include <rcutils/allocator.h> /* ROS2 工具库：自定义内存分配器接口 */
#include <rmw_microros/rmw_microros.h> /* micro-ROS 中间件：自定义传输、Agent 通信 */
#include <rmw_microros/time_sync.h>    /* micro-ROS 时间同步：与 Agent 对时，获取 ROS 时间戳 */
#include <std_msgs/msg/int32.h>        /* ROS2 标准消息：Int32 类型，用于 /stm32_ping 心跳 */
#include <geometry_msgs/msg/twist.h>   /* ROS2 几何消息：Twist 类型，用于 /cmd_vel 速度命令 */
#include <nav_msgs/msg/odometry.h>     /* ROS2 导航消息：Odometry 类型，用于 /odom 里程计 */
#include <uxr/client/transport.h>      /* micro-ROS 传输层：自定义传输 open/close/read/write 结构 */
#include "can_motor.h"      /* 后驱电机 CAN 控制模块：启停、速度换算、诊断 */
#include "can_steer.h"      /* 前轮转向电机 CAN 控制模块：手动模式、角度下发、EPS 反馈解析 */
#include "fdcan.h"          /* STM32 FDCAN 外设初始化句柄声明 */
#include "sbus.h"           /* SBUS 遥控接收模块：UART DMA 接收、帧解析、通道换算 */
#include "usart.h"          /* STM32 USART 外设句柄声明（huart1 用于 micro-ROS 传输） */
#include "atk_ms901m.h"     /* ATK-MS901M 姿态传感器驱动：初始化、姿态读取 */
#include "chassis_control.h" /* 底盘控制算法：cmd_vel 到电机目标换算、阿克曼横摆角速度解算 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;   /* 将 FreeRTOS 静态任务结构重命名为 CMSIS-OS 风格 */
typedef StaticQueue_t osStaticMessageQDef_t; /* 将 FreeRTOS 静态队列结构重命名为 CMSIS-OS 风格 */
/* USER CODE BEGIN PTD */
/* FDCAN接收队列元素：保存一帧CAN头和最多8字节数据。 */
typedef struct
{
  FDCAN_RxHeaderTypeDef header; /* CAN 帧头：ID、DLC、时间戳、滤波器匹配索引等 */
  uint8_t data[8];              /* CAN 数据段：最多 8 字节有效载荷 */
} CanRxMsg_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 最近一次后驱电机和转向电机CAN下发状态，供调试或反馈查看。 */
volatile HAL_StatusTypeDef g_can_motor_last_status = HAL_OK;    /* 后驱电机 CAN 发送状态（HAL_OK=成功，HAL_ERROR/HAL_BUSY/HAL_TIMEOUT=失败） */
volatile HAL_StatusTypeDef g_eps_steer_last_status = HAL_OK;    /* 转向电机 CAN 发送状态（同上） */
/* SBUS任务解析出的最近一帧遥控数据和控制量。 */
volatile SbusData_t g_sbus_data;          /* 最新 SBUS 原始通道数据（16 通道 11 位值 + failsafe/frame_lost 标志） */
volatile SbusControl_t g_sbus_control;    /* 由 SBUS 通道换算出的控制量（左右档位、油门百分比、转向百分比、急停布尔值） */
volatile uint8_t g_sbus_frame_ok = 0U;    /* 最新 SBUS 帧解析成功标志（1=有效帧，0=解析失败） */
volatile uint8_t g_sbus_failsafe = 0U;    /* SBUS 失控保护标志（1=遥控信号丢失，0=正常） */
volatile uint8_t g_sbus_estop_active = 0U; /* 急停激活标志（1=急停中，0=正常，由通道映射规则判定） */
/* MS901M初始化结果、最近姿态帧状态和接收计数。 */
volatile uint8_t g_ms901m_init_status = ATK_MS901M_ERROR;    /* MS901M 初始化结果（ATK_MS901M_EOK=成功，ATK_MS901M_ERROR=失败） */
volatile uint8_t g_ms901m_frame_ok = 0U;                      /* 最新姿态帧有效标志（1=本周期读取成功，0=读取失败） */
volatile uint32_t g_ms901m_rx_count = 0U;                     /* 累计成功读取姿态帧的计数（用于诊断通信稳定性） */
volatile atk_ms901m_attitude_data_t g_ms901m_attitude;        /* 最新 MS901M 姿态数据（欧拉角 pitch/roll/yaw + 四元数 + 角速度 + 加速度） */
/* /cmd_vel回调写入的最新线速度、角速度和接收时间戳。 */
volatile float g_chassis_cmd_linear_x = 0.0f;     /* /cmd_vel 目标线速度 (m/s)，由 cmd_vel_callback 写入，controlTask 读取 */
volatile float g_chassis_cmd_angular_z = 0.0f;    /* /cmd_vel 目标角速度 (rad/s)，由 cmd_vel_callback 写入，controlTask 读取 */
volatile uint32_t g_chassis_cmd_rx_count = 0U;    /* 累计收到的 /cmd_vel 消息数（用于诊断通信链路） */
volatile uint32_t g_chassis_cmd_last_tick = 0U;   /* 最近一次收到 /cmd_vel 时的系统 tick（用于 controlTask 超时检测） */
/* micro-ROS运行诊断计数：agent丢失预留、发布失败和executor spin错误。 */
volatile uint32_t g_microros_agent_lost_count = 0U;     /* Agent 连接丢失计数（预留，当前由 agent 端检测） */
volatile uint32_t g_microros_publish_fail_count = 0U;   /* 发布失败累计计数（成功时归零，持续失败表示通信异常） */
volatile uint32_t g_microros_spin_error_count = 0U;     /* executor.spin_some() 异常返回计数（非 RCL_RET_OK 且非 RCL_RET_TIMEOUT） */
/* micro-ROS发布器和消息对象，由defaultTask初始化后长期使用。 */
static rcl_publisher_t g_ping_publisher;    /* /stm32_ping 心跳发布器（Int32 类型，200ms 周期递增发布） */
static rcl_publisher_t g_odom_publisher;    /* /odom 里程计发布器（Odometry 类型，5Hz 周期发布） */
static std_msgs__msg__Int32 g_ping_msg;     /* 心跳消息实例（data 字段每次发布成功后自增 1） */
static nav_msgs__msg__Odometry g_odom_msg;  /* 里程计消息实例（包含 header、pose、twist 三个子结构） */

/* 里程计位姿积分变量。 */
static float g_odom_pose_x = 0.0f;       /* 里程计 X 坐标累积值 (m)，世界坐标系下 */
static float g_odom_pose_y = 0.0f;       /* 里程计 Y 坐标累积值 (m)，世界坐标系下 */
static float g_odom_pose_theta = 0.0f;   /* 里程计航向角累积值 (rad)，世界坐标系下 */
static uint32_t g_odom_last_tick = 0U;   /* 上一次里程计积分时的系统 tick（用于计算时间步长 dt） */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;                      /* defaultTask 线程 ID 句柄（osThreadNew 返回值） */
uint32_t defaultTaskBuffer[ 3000 ];                  /* defaultTask 线程栈内存（静态分配，3000 × 4 = 12KB） */
osStaticThreadDef_t defaultTaskControlBlock;         /* defaultTask 线程控制块（静态分配，保存线程状态） */
const osThreadAttr_t defaultTask_attributes = {      /* defaultTask 线程属性结构体 */
  .name = "defaultTask",                             /* 线程名称（用于调试和 FreeRTOS 任务列表） */
  .cb_mem = &defaultTaskControlBlock,                /* 指向静态线程控制块 */
  .cb_size = sizeof(defaultTaskControlBlock),        /* 线程控制块大小 */
  .stack_mem = &defaultTaskBuffer[0],                /* 指向静态栈内存首地址 */
  .stack_size = sizeof(defaultTaskBuffer),           /* 栈内存总大小（字节） */
  .priority = (osPriority_t) osPriorityNormal,       /* 线程优先级（Normal = 24，在 56 级系统中属中等） */
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;                      /* controlTask 线程 ID 句柄 */
uint32_t controlTaskBuffer[ 1024 ];                  /* controlTask 线程栈内存（1024 × 4 = 4KB） */
osStaticThreadDef_t controlTaskControlBlock;         /* controlTask 线程控制块 */
const osThreadAttr_t controlTask_attributes = {      /* controlTask 线程属性结构体 */
  .name = "controlTask",                             /* 线程名称 */
  .cb_mem = &controlTaskControlBlock,                /* 指向静态线程控制块 */
  .cb_size = sizeof(controlTaskControlBlock),        /* 线程控制块大小 */
  .stack_mem = &controlTaskBuffer[0],                /* 指向静态栈内存首地址 */
  .stack_size = sizeof(controlTaskBuffer),           /* 栈内存总大小 */
  .priority = (osPriority_t) osPriorityAboveNormal,  /* 线程优先级（AboveNormal > Normal，保证控制周期稳定） */
};
/* Definitions for canTask */
osThreadId_t canTaskHandle;                          /* canTask 线程 ID 句柄 */
uint32_t canTaskBuffer[ 1024 ];                      /* canTask 线程栈内存（1024 × 4 = 4KB） */
osStaticThreadDef_t canTaskControlBlock;             /* canTask 线程控制块 */
const osThreadAttr_t canTask_attributes = {          /* canTask 线程属性结构体 */
  .name = "canTask",                                 /* 线程名称 */
  .cb_mem = &canTaskControlBlock,                    /* 指向静态线程控制块 */
  .cb_size = sizeof(canTaskControlBlock),            /* 线程控制块大小 */
  .stack_mem = &canTaskBuffer[0],                    /* 指向静态栈内存首地址 */
  .stack_size = sizeof(canTaskBuffer),               /* 栈内存总大小 */
  .priority = (osPriority_t) osPriorityAboveNormal,  /* 线程优先级（高于 Normal，确保 CAN 消息及时处理） */
};
/* Definitions for encoderTask */
osThreadId_t encoderTaskHandle;                      /* encoderTask 线程 ID 句柄 */
uint32_t encoderTaskBuffer[ 512 ];                   /* encoderTask 线程栈内存（512 × 4 = 2KB） */
osStaticThreadDef_t encoderTaskControlBlock;         /* encoderTask 线程控制块 */
const osThreadAttr_t encoderTask_attributes = {      /* encoderTask 线程属性结构体 */
  .name = "encoderTask",                             /* 线程名称 */
  .cb_mem = &encoderTaskControlBlock,                /* 指向静态线程控制块 */
  .cb_size = sizeof(encoderTaskControlBlock),        /* 线程控制块大小 */
  .stack_mem = &encoderTaskBuffer[0],                /* 指向静态栈内存首地址 */
  .stack_size = sizeof(encoderTaskBuffer),           /* 栈内存总大小 */
  .priority = (osPriority_t) osPriorityNormal,       /* 线程优先级（预留任务，Normal 即可） */
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;                          /* imuTask 线程 ID 句柄 */
uint32_t imuTaskBuffer[ 1024 ];                      /* imuTask 线程栈内存（1024 × 4 = 4KB） */
osStaticThreadDef_t imuTaskControlBlock;             /* imuTask 线程控制块 */
const osThreadAttr_t imuTask_attributes = {          /* imuTask 线程属性结构体 */
  .name = "imuTask",                                 /* 线程名称 */
  .cb_mem = &imuTaskControlBlock,                    /* 指向静态线程控制块 */
  .cb_size = sizeof(imuTaskControlBlock),            /* 线程控制块大小 */
  .stack_mem = &imuTaskBuffer[0],                    /* 指向静态栈内存首地址 */
  .stack_size = sizeof(imuTaskBuffer),               /* 栈内存总大小 */
  .priority = (osPriority_t) osPriorityNormal,       /* 线程优先级（10ms 周期读取姿态，Normal 即可） */
};
/* Definitions for sbusTask */
osThreadId_t sbusTaskHandle;                         /* sbusTask 线程 ID 句柄 */
uint32_t sbusTaskBuffer[ 1024 ];                     /* sbusTask 线程栈内存（1024 × 4 = 4KB） */
osStaticThreadDef_t sbusTaskControlBlock;            /* sbusTask 线程控制块 */
const osThreadAttr_t sbusTask_attributes = {         /* sbusTask 线程属性结构体 */
  .name = "sbusTask",                                /* 线程名称 */
  .cb_mem = &sbusTaskControlBlock,                   /* 指向静态线程控制块 */
  .cb_size = sizeof(sbusTaskControlBlock),           /* 线程控制块大小 */
  .stack_mem = &sbusTaskBuffer[0],                   /* 指向静态栈内存首地址 */
  .stack_size = sizeof(sbusTaskBuffer),              /* 栈内存总大小 */
  .priority = (osPriority_t) osPriorityNormal,       /* 线程优先级（10ms 周期解析 SBUS 帧，Normal 即可） */
};
/* Definitions for batteryTask */
osThreadId_t batteryTaskHandle;                      /* batteryTask 线程 ID 句柄 */
uint32_t batteryTaskBuffer[ 512 ];                   /* batteryTask 线程栈内存（512 × 4 = 2KB） */
osStaticThreadDef_t batteryTaskControlBlock;         /* batteryTask 线程控制块 */
const osThreadAttr_t batteryTask_attributes = {      /* batteryTask 线程属性结构体 */
  .name = "batteryTask",                             /* 线程名称 */
  .cb_mem = &batteryTaskControlBlock,                /* 指向静态线程控制块 */
  .cb_size = sizeof(batteryTaskControlBlock),        /* 线程控制块大小 */
  .stack_mem = &batteryTaskBuffer[0],                /* 指向静态栈内存首地址 */
  .stack_size = sizeof(batteryTaskBuffer),           /* 栈内存总大小 */
  .priority = (osPriority_t) osPriorityLow,          /* 线程优先级（低优先级预留任务） */
};
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;                          /* gpsTask 线程 ID 句柄 */
uint32_t gpsTaskBuffer[ 1024 ];                      /* gpsTask 线程栈内存（1024 × 4 = 4KB） */
osStaticThreadDef_t gpsTaskControlBlock;             /* gpsTask 线程控制块 */
const osThreadAttr_t gpsTask_attributes = {          /* gpsTask 线程属性结构体 */
  .name = "gpsTask",                                 /* 线程名称 */
  .cb_mem = &gpsTaskControlBlock,                    /* 指向静态线程控制块 */
  .cb_size = sizeof(gpsTaskControlBlock),            /* 线程控制块大小 */
  .stack_mem = &gpsTaskBuffer[0],                    /* 指向静态栈内存首地址 */
  .stack_size = sizeof(gpsTaskBuffer),               /* 栈内存总大小 */
  .priority = (osPriority_t) osPriorityLow,          /* 线程优先级（低优先级预留任务） */
};
/* Definitions for lidarTask */
osThreadId_t lidarTaskHandle;                        /* lidarTask 线程 ID 句柄 */
uint32_t lidarTaskBuffer[ 1024 ];                    /* lidarTask 线程栈内存（1024 × 4 = 4KB） */
osStaticThreadDef_t lidarTaskControlBlock;           /* lidarTask 线程控制块 */
const osThreadAttr_t lidarTask_attributes = {        /* lidarTask 线程属性结构体 */
  .name = "lidarTask",                               /* 线程名称 */
  .cb_mem = &lidarTaskControlBlock,                  /* 指向静态线程控制块 */
  .cb_size = sizeof(lidarTaskControlBlock),          /* 线程控制块大小 */
  .stack_mem = &lidarTaskBuffer[0],                  /* 指向静态栈内存首地址 */
  .stack_size = sizeof(lidarTaskBuffer),             /* 栈内存总大小 */
  .priority = (osPriority_t) osPriorityLow,          /* 线程优先级（低优先级预留任务） */
};
/* Definitions for canRxQueue */
osMessageQueueId_t canRxQueueHandle;                      /* CAN 接收队列的 CMSIS-OS 句柄（供 ISR 和生产任务使用） */
uint8_t canRxQueueBuffer[ 32 * sizeof( CanRxMsg_t ) ];   /* CAN 接收队列数据缓冲区（最多缓存 32 帧） */
osStaticMessageQDef_t canRxQueueControlBlock;             /* CAN 接收队列静态控制块 */
const osMessageQueueAttr_t canRxQueue_attributes = {      /* CAN 接收队列属性结构体 */
  .name = "canRxQueue",                                   /* 队列名称 */
  .cb_mem = &canRxQueueControlBlock,                      /* 指向静态队列控制块 */
  .cb_size = sizeof(canRxQueueControlBlock),              /* 队列控制块大小 */
  .mq_mem = &canRxQueueBuffer,                            /* 指向队列数据缓冲区首地址 */
  .mq_size = sizeof(canRxQueueBuffer)                     /* 队列数据缓冲区总大小 */
};
/* Definitions for cmdVelQueue */
osMessageQueueId_t cmdVelQueueHandle;                      /* /cmd_vel 命令队列的 CMSIS-OS 句柄（预留队列，当前未使用） */
uint8_t cmdVelQueueBuffer[ 4 * sizeof( uint32_t ) ];      /* /cmd_vel 命令队列数据缓冲区（最多缓存 4 个 uint32_t） */
osStaticMessageQDef_t cmdVelQueueControlBlock;             /* /cmd_vel 命令队列静态控制块 */
const osMessageQueueAttr_t cmdVelQueue_attributes = {      /* /cmd_vel 命令队列属性结构体 */
  .name = "cmdVelQueue",                                   /* 队列名称 */
  .cb_mem = &cmdVelQueueControlBlock,                      /* 指向静态队列控制块 */
  .cb_size = sizeof(cmdVelQueueControlBlock),              /* 队列控制块大小 */
  .mq_mem = &cmdVelQueueBuffer,                            /* 指向队列数据缓冲区首地址 */
  .mq_size = sizeof(cmdVelQueueBuffer)                     /* 队列数据缓冲区总大小 */
};
/* Definitions for sbusQueue */
osMessageQueueId_t sbusQueueHandle;                        /* SBUS 遥控队列的 CMSIS-OS 句柄（预留队列，当前未使用） */
uint8_t sbusQueueBuffer[ 4 * sizeof( uint32_t ) ];        /* SBUS 遥控队列数据缓冲区（最多缓存 4 个 uint32_t） */
osStaticMessageQDef_t sbusQueueControlBlock;               /* SBUS 遥控队列静态控制块 */
const osMessageQueueAttr_t sbusQueue_attributes = {        /* SBUS 遥控队列属性结构体 */
  .name = "sbusQueue",                                     /* 队列名称 */
  .cb_mem = &sbusQueueControlBlock,                        /* 指向静态队列控制块 */
  .cb_size = sizeof(sbusQueueControlBlock),                /* 队列控制块大小 */
  .mq_mem = &sbusQueueBuffer,                              /* 指向队列数据缓冲区首地址 */
  .mq_size = sizeof(sbusQueueBuffer)                       /* 队列数据缓冲区总大小 */
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);   /* 自定义传输打开函数：初始化 UART1 DMA 接收 */
bool cubemx_transport_close(struct uxrCustomTransport * transport);  /* 自定义传输关闭函数：停止 UART1 DMA 接收 */

size_t cubemx_transport_write(                           /* 自定义传输写函数：通过 UART1 DMA 发送数据到 Agent */
struct uxrCustomTransport * transport,                   /* micro-ROS 传输对象指针 */
const uint8_t * buf,                                     /* 待发送数据缓冲区 */
size_t len,                                              /* 待发送数据长度（字节） */
uint8_t * err);                                          /* 输出错误码指针 */

size_t cubemx_transport_read(                            /* 自定义传输读函数：从 UART1 DMA 接收 Agent 发来的数据 */
struct uxrCustomTransport * transport,                   /* micro-ROS 传输对象指针 */
uint8_t * buf,                                           /* 接收数据缓冲区 */
size_t len,                                              /* 期望读取的最大长度（字节） */
int timeout,                                             /* 读取超时时间（毫秒） */
uint8_t * err);                                          /* 输出错误码指针 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);   /* 自定义传输打开（重复声明，与上方一致） */
bool cubemx_transport_close(struct uxrCustomTransport * transport);  /* 自定义传输关闭（重复声明，与上方一致） */

size_t cubemx_transport_write(                           /* 自定义传输写（重复声明，与上方一致） */
struct uxrCustomTransport * transport,
const uint8_t * buf,
size_t len,
uint8_t * err);

size_t cubemx_transport_read(                            /* 自定义传输读（重复声明，与上方一致） */
struct uxrCustomTransport * transport,
uint8_t * buf,
size_t len,
int timeout,
uint8_t * err);

void * microros_allocate(size_t size, void * state);                        /* micro-ROS 内存分配回调：从 FreeRTOS heap 分配 */
void microros_deallocate(void * pointer, void * state);                     /* micro-ROS 内存释放回调：归还到 FreeRTOS heap */
void * microros_reallocate(void * pointer, size_t size, void * state);     /* micro-ROS 内存重分配回调：调整已分配块大小 */
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state); /* micro-ROS 零初始化分配回调 */

static void ping_timer_callback(rcl_timer_t *timer, int64_t last_call_time);   /* /stm32_ping 定时器回调：发布递增 Int32 心跳 */
static void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time);   /* /odom 定时器回调：积分位姿并发布里程计消息 */
static void cmd_vel_callback(const void *msgin);                                /* /cmd_vel 订阅回调：缓存上位机速度命令到全局变量 */
static void can_rx_process(const CanRxMsg_t *msg);                              /* CAN 接收帧处理：解析后驱反馈 0x186 和 EPS 反馈 0x18F */


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);   /* micro-ROS 主通信任务入口（函数声明） */
void StartControlTask(void *argument);   /* 10ms 底盘控制任务入口（函数声明） */
void StartCanTask(void *argument);       /* CAN 后台处理任务入口（函数声明） */
void StartEncoderTask(void *argument);   /* 编码器采集任务入口（函数声明，预留） */
void StartImuTask(void *argument);       /* IMU 姿态采集任务入口（函数声明） */
void StartSbusTask(void *argument);      /* SBUS 遥控接收任务入口（函数声明） */
void StartBatteryTask(void *argument);   /* 电池监测任务入口（函数声明，预留） */
void StartGpsTask(void *argument);       /* GPS 定位任务入口（函数声明，预留） */
void StartLidarTask(void *argument);     /* 激光雷达任务入口（函数声明，预留） */

void MX_FREERTOS_Init(void); /*  FreeRTOS 总初始化函数：创建所有队列和线程（MISRA C 2004 rule 8.1 要求前置声明） */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {      /* FreeRTOS 初始化函数：由 main.c 在调度器启动前调用 */
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* 当前没有创建互斥锁。 */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* 当前没有创建信号量。 */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* 当前没有创建CMSIS-OS定时器；micro-ROS定时器在defaultTask内初始化。 */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canRxQueue */
  canRxQueueHandle = osMessageQueueNew (32, sizeof(CanRxMsg_t), &canRxQueue_attributes); /* 创建 CAN 接收队列：容量 32，元素大小 = CanRxMsg_t */

  /* creation of cmdVelQueue */
  cmdVelQueueHandle = osMessageQueueNew (4, sizeof(uint32_t), &cmdVelQueue_attributes);  /* 创建 /cmd_vel 队列：容量 4，元素大小 = uint32_t（预留） */

  /* creation of sbusQueue */
  sbusQueueHandle = osMessageQueueNew (4, sizeof(uint32_t), &sbusQueue_attributes);      /* 创建 SBUS 队列：容量 4，元素大小 = uint32_t（预留） */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 队列已在上方创建，这里没有额外队列。 */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);   /* 创建 micro-ROS 主通信线程 */

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(StartControlTask, NULL, &controlTask_attributes);   /* 创建 10ms 底盘控制线程 */

  /* creation of canTask */
  canTaskHandle = osThreadNew(StartCanTask, NULL, &canTask_attributes);               /* 创建 CAN 后台处理线程 */

  /* creation of encoderTask */
  encoderTaskHandle = osThreadNew(StartEncoderTask, NULL, &encoderTask_attributes);   /* 创建编码器采集线程（预留） */

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);               /* 创建 IMU 姿态采集线程 */

  /* creation of sbusTask */
  sbusTaskHandle = osThreadNew(StartSbusTask, NULL, &sbusTask_attributes);            /* 创建 SBUS 遥控接收线程 */

  /* creation of batteryTask */
  batteryTaskHandle = osThreadNew(StartBatteryTask, NULL, &batteryTask_attributes);   /* 创建电池监测线程（预留） */

  /* creation of gpsTask */
  gpsTaskHandle = osThreadNew(StartGpsTask, NULL, &gpsTask_attributes);               /* 创建 GPS 定位线程（预留） */

  /* creation of lidarTask */
  lidarTaskHandle = osThreadNew(StartLidarTask, NULL, &lidarTask_attributes);         /* 创建激光雷达线程（预留） */

  /* USER CODE BEGIN RTOS_THREADS */
  /* 线程已在上方创建，这里没有额外线程。 */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* 当前没有创建事件标志。 */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief       micro-ROS通信与消息调度任务
  * @param       argument: 任务参数，当前未使用
  * @retval      无
  * @note        使用UART1自定义传输连接agent，初始化节点、发布器、订阅器和定时器，
  *              executor循环处理/cmd_vel订阅，并周期发布/stm32_ping和/chassis_feedback。
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void) argument;   /* 消除"未使用参数"编译器警告 */

  /* micro-ROS使用UART1上的自定义传输函数连接agent。 */
  rmw_uros_set_custom_transport(    /* 设置 micro-ROS 自定义传输层：通过 UART1 与 Agent 通信 */
  true,                              /* 参数1 framing: true = 使用嵌入式 RTPS 帧封装 */
  (void *) &huart1,                 /* 参数2 args: 传递 UART1 句柄指针给 open/close/read/write */
  cubemx_transport_open,            /* 参数3 open: 传输打开回调函数指针 */
  cubemx_transport_close,           /* 参数4 close: 传输关闭回调函数指针 */
  cubemx_transport_write,           /* 参数5 write: 传输写回调函数指针（阻塞发送） */
  cubemx_transport_read);           /* 参数6 read: 传输读回调函数指针（带超时接收） */

  /* 将micro-ROS内存分配切换到工程提供的FreeRTOS分配器。 */
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator(); /* 获取一个全零初始化的分配器结构体 */

  freeRTOS_allocator.allocate = microros_allocate;           /* 绑定内存分配函数：调用 pvPortMalloc */
  freeRTOS_allocator.deallocate = microros_deallocate;       /* 绑定内存释放函数：调用 vPortFree */
  freeRTOS_allocator.reallocate = microros_reallocate;       /* 绑定内存重分配函数：malloc + memcpy + free */
  freeRTOS_allocator.zero_allocate = microros_zero_allocate; /* 绑定零初始化分配函数：calloc 等效实现 */

  if (!rcutils_set_default_allocator(&freeRTOS_allocator))   /* 将自定义分配器设为 ROS2 全局默认分配器 */
  {
    for (;;)           /* 分配器设置失败 → 死循环卡死（系统级错误，无法恢复） */
    {
      osDelay(1000U);  /* 每 1000 tick (1秒) 空转一次，降低功耗 */
    }
  }

  rcl_allocator_t allocator = rcl_get_default_allocator(); /* 获取当前默认分配器的副本，供后续 rcl API 使用 */

  static rclc_support_t support;               /* rclc 支持对象：封装 rcl 初始化上下文和分配器 */
  static rcl_node_t node;                      /* ROS2 节点对象：名为 "stm32_h743_node"，无命名空间 */
  static rcl_subscription_t cmd_vel_subscriber; /* /cmd_vel 订阅器对象 */
  static geometry_msgs__msg__Twist cmd_vel_msg; /* /cmd_vel 消息实例（回调时由 executor 自动填充） */
  static rcl_timer_t ping_timer;               /* /stm32_ping 定时器对象（200ms 周期） */
  static rcl_timer_t odom_timer;               /* /odom 定时器对象（200ms 周期，即 5Hz） */
  static rclc_executor_t executor;             /* rclc executor 对象：单线程驱动所有回调和定时器 */

  /* 以下micro-ROS对象初始化失败时会清除错误并等待重试。 */
  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) /* 初始化 rclc 支持层（context=0 使用默认） */
  {
    rcl_reset_error();   /* 清除 rcl 错误状态，防止后续 API 因残留错误而失败 */
    osDelay(500U);       /* 等待 500 tick (500ms) 后重试 */
  }

  while (rclc_node_init_default(&node, "stm32_h743_node", "", &support) != RCL_RET_OK) /* 创建默认节点 */
  {
    rcl_reset_error();   /* 清除错误状态 */
    osDelay(500U);       /* 500ms 后重试 */
  }

  while (rclc_publisher_init_default(    /* 初始化 /stm32_ping 发布器（Int32 类型） */
  &g_ping_publisher,                     /* 发布器对象指针 */
  &node,                                 /* 所属节点 */
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), /* 消息类型支持（由代码生成器提供） */
  "/stm32_ping") != RCL_RET_OK)          /* 主题名称：/stm32_ping */
  {
    rcl_reset_error();   /* 清除错误状态 */
    osDelay(500U);       /* 500ms 后重试 */
  }

  while (rclc_publisher_init_default(    /* 初始化 /odom 发布器（Odometry 类型） */
  &g_odom_publisher,                     /* 发布器对象指针 */
  &node,                                 /* 所属节点 */
  ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), /* 消息类型支持 */
  "/odom") != RCL_RET_OK)               /* 主题名称：/odom */
  {
    rcl_reset_error();   /* 清除错误状态 */
    osDelay(500U);       /* 500ms 后重试 */
  }

  while (rclc_subscription_init_default( /* 初始化 /cmd_vel 订阅器（Twist 类型） */
  &cmd_vel_subscriber,                   /* 订阅器对象指针 */
  &node,                                 /* 所属节点 */
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), /* 消息类型支持 */
  "/cmd_vel") != RCL_RET_OK)             /* 主题名称：/cmd_vel */
  {
    rcl_reset_error();   /* 清除错误状态 */
    osDelay(500U);       /* 500ms 后重试 */
  }

  g_ping_msg.data = 0;       /* 心跳计数器初始化为 0 */
  cmd_vel_msg.linear.x = 0.0;  /* /cmd_vel 线速度 X 分量初始化为 0 */
  cmd_vel_msg.linear.y = 0.0;  /* /cmd_vel 线速度 Y 分量初始化为 0（本项目不使用） */
  cmd_vel_msg.linear.z = 0.0;  /* /cmd_vel 线速度 Z 分量初始化为 0（本项目不使用） */
  cmd_vel_msg.angular.x = 0.0; /* /cmd_vel 角速度 X 分量初始化为 0（本项目不使用） */
  cmd_vel_msg.angular.y = 0.0; /* /cmd_vel 角速度 Y 分量初始化为 0（本项目不使用） */
  cmd_vel_msg.angular.z = 0.0; /* /cmd_vel 角速度 Z 分量初始化为 0 */

  /* 里程计消息初始化。 */
  g_odom_msg.header.frame_id.data = (char *)"odom";        /* 里程计 frame_id 设为 "odom"（世界坐标系） */
  g_odom_msg.header.frame_id.size = 4;                      /* frame_id 字符串当前有效长度 = 4（"odom" 不含 '\0'） */
  g_odom_msg.header.frame_id.capacity = 5;                  /* frame_id 字符串缓冲区容量 = 5（预留 '\0' 位置） */
  g_odom_msg.child_frame_id.data = (char *)"base_link";     /* 子坐标系设为 "base_link"（机器人基座坐标系） */
  g_odom_msg.child_frame_id.size = 9;                       /* child_frame_id 字符串有效长度 = 9 */
  g_odom_msg.child_frame_id.capacity = 10;                  /* child_frame_id 字符串缓冲区容量 = 10 */

  while (rclc_timer_init_default(    /* 初始化 ping 定时器（200ms 周期 = 5Hz） */
  &ping_timer,                        /* 定时器对象指针 */
  &support,                           /* rclc 支持对象 */
  RCL_MS_TO_NS(200),                  /* 周期：200ms 转换为纳秒（200,000,000 ns） */
  ping_timer_callback) != RCL_RET_OK) /* 定时器到期时调用的回调函数 */
  {
    rcl_reset_error();   /* 清除错误状态 */
    osDelay(500U);       /* 500ms 后重试 */
  }

  while (rclc_timer_init_default(    /* 初始化 odom 定时器（200ms 周期 = 5Hz） */
  &odom_timer,                        /* 定时器对象指针 */
  &support,                           /* rclc 支持对象 */
  RCL_MS_TO_NS(200),                  /* 周期：200ms 转换为纳秒 */
  odom_timer_callback) != RCL_RET_OK) /* 定时器到期时调用的回调函数（里程计积分 + 发布） */
  {
    rcl_reset_error();   /* 清除错误状态 */
    osDelay(500U);       /* 500ms 后重试 */
  }

  /* executor容量为3：一个/cmd_vel订阅回调和两个定时器回调。 */
  while (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) /* 初始化 executor，句柄数 = 3 */
  {
    rcl_reset_error();   /* 清除错误状态 */
    osDelay(500U);       /* 500ms 后重试 */
  }

  while (rclc_executor_add_subscription( /* 将 /cmd_vel 订阅器注册到 executor */
  &executor,                              /* executor 对象指针 */
  &cmd_vel_subscriber,                    /* 订阅器对象指针 */
  &cmd_vel_msg,                           /* 消息实例指针（有新数据时自动填充） */
  cmd_vel_callback,                       /* 收到新消息时的回调函数 */
  ON_NEW_DATA) != RCL_RET_OK)             /* 触发策略：仅在新数据到达时调用回调 */
  {
    rcl_reset_error();   /* 清除错误状态 */
    osDelay(500U);       /* 500ms 后重试 */
  }

  while (rclc_executor_add_timer(&executor, &ping_timer) != RCL_RET_OK) /* 将 ping 定时器注册到 executor */
  {
    rcl_reset_error();   /* 清除错误状态 */
    osDelay(500U);       /* 500ms 后重试 */
  }

  while (rclc_executor_add_timer(&executor, &odom_timer) != RCL_RET_OK) /* 将 odom 定时器注册到 executor */
  {
    rcl_reset_error();   /* 清除错误状态 */
    osDelay(500U);       /* 500ms 后重试 */
  }

  /* 通过串口与 Agent 同步时间，使 odom 时间戳与 ROS 时间对齐。 */
  while (rmw_uros_sync_session(1000) != RMW_RET_OK) /* 与 Agent 时间同步，超时 1000ms */
  {
    rcl_reset_error();   /* 清除错误状态 */
    osDelay(100U);       /* 100ms 后重试 */
  }

  /* 主循环驱动micro-ROS executor，最多等待50ms处理一次事件。 */
  for (;;)   /* FreeRTOS 任务永不返回，无限循环 */
  {
    rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)); /* 驱动 executor 处理事件，超时 50ms */

    if ((spin_ret != RCL_RET_OK) && (spin_ret != RCL_RET_TIMEOUT)) /* 返回值既非 OK 也非 TIMEOUT = 异常 */
    {
      g_microros_spin_error_count++;  /* 累加 executor 异常计数，用于诊断 */
      rcl_reset_error();              /* 清除错误状态，防止下次 spin 被阻塞 */
    }

    osDelay(10U);  /* 延时 10 tick (10ms)，让出 CPU 给其他低优先级任务 */
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief       10ms底盘控制任务，根据/cmd_vel驱动后驱和转向。
* @param       argument: 任务参数，当前未使用
* @retval      无
* @note        超过500ms没有收到/cmd_vel时清零速度命令，
*              正常情况下调用Chassis_ControlFromCmdVel()换算并下发执行目标。
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  (void)argument;  /* 消除"未使用参数"编译器警告 */

  /*
   * 控制任务启动时先打开 FDCAN1，并把后驱和转向都置到安全零值。
   * 后续 10ms 循环只负责根据 /cmd_vel 周期刷新目标。
   */
  CanMotor_Init();                           /* 初始化后驱电机 CAN 通信参数（速度换算系数、限幅等） */
  g_can_motor_last_status = CanMotor_Stop(); /* 发送 CAN 停止命令给后驱电机，保存发送状态 */
  g_eps_steer_last_status = EpsSteer_SetManualMode(); /* 设置 EPS 转向为手动模式（允许 MCU 下发角度命令） */
  g_eps_steer_last_status = EpsSteer_SetZero();      /* 将前轮转向角置零（回正），保存发送状态 */

  /*
   * 10ms 底盘控制循环。
   * micro-ROS 的 /cmd_vel 回调负责更新 g_chassis_cmd_linear_x 和 g_chassis_cmd_angular_z，
   * 这里统一转换成后驱电机目标速度和前轮转向目标角度。
   */
  for (;;)   /* 控制循环永不退出 */
  {
    uint32_t now = osKernelGetTickCount(); /* 获取当前系统 tick 计数（1 tick = 1ms） */

    /* /cmd_vel超时或从未收到命令时，主动清零目标避免底盘继续运动。 */
    if ((g_chassis_cmd_last_tick == 0U) || ((now - g_chassis_cmd_last_tick) > pdMS_TO_TICKS(500))) /* 从未收到命令 或 超过500ms没有更新 */
    {
      g_chassis_cmd_linear_x = 0.0f;   /* 线速度清零（停止前进/后退） */
      g_chassis_cmd_angular_z = 0.0f;  /* 角速度清零（停止转向） */
    }

    /* 将最新线速度和角速度转换为后驱速度、前轮转角，并通过底层接口下发。 */
    Chassis_ControlFromCmdVel(g_chassis_cmd_linear_x, g_chassis_cmd_angular_z); /* 阿克曼模型逆向解算 + CAN 下发 */
    /* 保存本周期底层CAN发送状态，便于调试和/chassis_feedback观察。 */
    g_can_motor_last_status = g_drive_motor_can_status;  /* 缓存后驱电机 CAN 发送状态 */
    g_eps_steer_last_status = g_steer_motor_can_status;  /* 缓存转向电机 CAN 发送状态 */
    CanMotor_UpdateDiagnostics();  /* 更新后驱电机诊断信息（超时检测、通信状态统计） */
    osDelay(10U);                  /* 延时 10 tick (10ms)，实现 100Hz 控制频率 */
  }

  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartCanTask */
/**
* @brief CAN后台任务预留，当前没有收发处理逻辑。
* @param argument: 任务参数，当前未使用
* @retval 无
*/
/* USER CODE END Header_StartCanTask */
void StartCanTask(void *argument)
{
  /* USER CODE BEGIN StartCanTask */
  (void)argument;  /* 消除"未使用参数"编译器警告 */

  /* 当前任务尚无具体业务，周期延时保持线程存活。 */
  for(;;)   /* 无限循环阻塞等待 CAN 消息 */
  {
    CanRxMsg_t rx_msg;  /* 栈上分配 CAN 接收消息临时变量 */

    if (osMessageQueueGet(canRxQueueHandle, &rx_msg, NULL, osWaitForever) == osOK) /* 阻塞等待 CAN 队列有新消息 */
    {
      can_rx_process(&rx_msg); /* 处理收到的 CAN 帧（解析 0x186 后驱反馈 / 0x18F EPS 反馈） */
    }
  }
  /* USER CODE END StartCanTask */
}

/* USER CODE BEGIN Header_StartEncoderTask */
/**
* @brief 编码器采集任务预留，当前没有读取编码器逻辑。
* @param argument: 任务参数，当前未使用
* @retval 无
*/
/* USER CODE END Header_StartEncoderTask */
void StartEncoderTask(void *argument)
{
  /* USER CODE BEGIN StartEncoderTask */
  /* 当前任务尚无具体业务，周期延时保持线程存活。 */
  for(;;)         /* 占位循环 */
  {
    osDelay(1000U); /* 延时 1000 tick (1秒)，保持线程不退出 */
  }
  /* USER CODE END StartEncoderTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief 初始化MS901M姿态传感器，并周期读取姿态数据。
* @param argument: 任务参数，当前未使用
* @retval 无
*/
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
  (void)argument;  /* 消除"未使用参数"编译器警告 */

  /* MS901M按115200波特率初始化，初始化结果保存到全局状态。 */
  g_ms901m_init_status = atk_ms901m_init(115200U); /* 初始化 MS901M，波特率 115200，结果存全局变量 */

  for (;;)   /* 10ms 周期姿态读取循环 */
  {
    atk_ms901m_attitude_data_t attitude; /* 栈上分配姿态数据临时变量 */

    /* 每10ms尝试读取一次姿态，单次读取最多等待5ms。 */
    if (atk_ms901m_get_attitude(&attitude, 5U) == ATK_MS901M_EOK) /* 尝试读取姿态，超时 5ms */
    {
      /* 读取成功后发布最新姿态，并累加有效帧计数。 */
      g_ms901m_attitude = attitude;   /* 拷贝最新姿态数据到全局变量（供 odom 回调等使用） */
      g_ms901m_frame_ok = 1U;         /* 标记本周期姿态有效 */
      g_ms901m_rx_count++;            /* 累加有效帧计数（诊断用） */
    }
    else
    {
      /* 本周期没有拿到有效姿态帧，只清状态，不清最后一次姿态值。 */
      g_ms901m_frame_ok = 0U; /* 标记本周期姿态无效（保留上次有效值供上层容错） */
    }

    osDelay(10U); /* 延时 10 tick (10ms)，实现 100Hz 读取频率 */
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartSbusTask */
/**
* @brief 启动SBUS接收，解析新遥控帧并更新手动控制状态。
* @param argument: 任务参数，当前未使用
* @retval 无
*/
/* USER CODE END Header_StartSbusTask */
void StartSbusTask(void *argument)
{
  /* USER CODE BEGIN StartSbusTask */
  (void)argument;  /* 消除"未使用参数"编译器警告 */

  /* UART5按SBUS真实接收配置启动，底层会优先使用DMA接收整帧。 */
  SBUS_Init();  /* 初始化 SBUS：UART5 配置为 100kbps/8E2 并启动 DMA 循环接收 */

  uint32_t last_rx_count = SBUS_GetRxFrameCount(); /* 记录当前 SBUS 帧计数作为基线 */

  for (;;)   /* 10ms 周期 SBUS 帧检查循环 */
  {
    uint32_t rx_count = SBUS_GetRxFrameCount(); /* 获取最新的 SBUS 帧计数 */

    /* 只有帧计数变化时才解析，避免重复处理同一帧SBUS数据。 */
    if (rx_count != last_rx_count) /* 帧计数变化 = 有新帧到达 */
    {
      SbusData_t data;          /* 栈上分配 SBUS 原始数据临时变量 */
      SbusControl_t ctrl;       /* 栈上分配 SBUS 控制量临时变量 */
      const uint8_t *frame = SBUS_GetRxBuffer(); /* 获取 DMA 接收缓冲区指针（指向 25 字节 SBUS 帧） */

      last_rx_count = rx_count; /* 更新本地帧计数，避免重复处理同一帧 */

      /* 帧头和打包格式检查通过后，将16通道数据转换为控制量。 */
      if (SBUS_TryParseFrame(frame, &data)) /* 解析 SBUS 帧：验证帧头、尾标志，提取 16 通道和标志位 */
      {
        SBUS_ConvertToControl(&data, &ctrl); /* 将 16 通道原始值 (0~2047) 换算为控制量（档位、百分比、急停） */

        /* 发布最新SBUS原始通道、控制量、failsafe和急停状态。 */
        g_sbus_data = data;                /* 更新全局 SBUS 原始数据 */
        g_sbus_control = ctrl;             /* 更新全局 SBUS 控制量 */
        g_sbus_frame_ok = 1U;              /* 标记当前帧解析有效 */
        g_sbus_failsafe = data.failsafe;    /* 更新遥控失控标志（0=正常，1=信号丢失） */
        g_sbus_estop_active = ctrl.estop;   /* 更新急停激活状态（0=正常，1=急停激活） */

      }
      else
      {
        /* 新帧解析失败时仅标记无效，保留上一帧控制状态供上层自行处理。 */
        g_sbus_frame_ok = 0U; /* 标记当前帧无效（上层可根据此标志决定是否使用旧数据） */
      }
    }

    osDelay(10); /* 延时 10 tick (10ms)，100Hz 检查频率 */
  }

  /* USER CODE END StartSbusTask */
}

/* USER CODE BEGIN Header_StartBatteryTask */
/**
* @brief 电池监测任务预留，当前没有采样或告警逻辑。
* @param argument: 任务参数，当前未使用
* @retval 无
*/
/* USER CODE END Header_StartBatteryTask */
void StartBatteryTask(void *argument)
{
  /* USER CODE BEGIN StartBatteryTask */
  /* 当前任务尚无具体业务，周期延时保持线程存活。 */
  for(;;)         /* 占位循环 */
  {
    osDelay(1000U); /* 延时 1000 tick (1秒)，保持线程不退出 */
  }
  /* USER CODE END StartBatteryTask */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
* @brief GPS任务预留，当前没有定位数据读取逻辑。
* @param argument: 任务参数，当前未使用
* @retval 无
*/
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
  /* USER CODE BEGIN StartGpsTask */
  /* 当前任务尚无具体业务，周期延时保持线程存活。 */
  for(;;)         /* 占位循环 */
  {
    osDelay(1000U); /* 延时 1000 tick (1秒)，保持线程不退出 */
  }
  /* USER CODE END StartGpsTask */
}

/* USER CODE BEGIN Header_StartLidarTask */
/**
* @brief 雷达任务预留，当前没有雷达数据读取逻辑。
* @param argument: 任务参数，当前未使用
* @retval 无
*/
/* USER CODE END Header_StartLidarTask */
void StartLidarTask(void *argument)
{
  /* USER CODE BEGIN StartLidarTask */
  /* 当前任务尚无具体业务，周期延时保持线程存活。 */
  for(;;)         /* 占位循环 */
  {
    osDelay(1000U); /* 延时 1000 tick (1秒)，保持线程不退出 */
  }
  /* USER CODE END StartLidarTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void can_rx_process(const CanRxMsg_t *msg) /* CAN 接收帧处理函数：按 CAN ID 分发到不同解析逻辑 */
{
  if (msg == NULL)   /* 空指针保护 */
  {
    return;          /* 直接返回，不做处理 */
  }

  if (msg->header.IdType != FDCAN_STANDARD_ID) /* 只处理标准帧（11 位 ID），忽略扩展帧（29 位 ID） */
  {
    return;  /* 非标准帧直接丢弃 */
  }

  /* 后驱电机反馈帧 0x186：解析实际线速度。 */
  if ((msg->header.Identifier == CAN_MOTOR_FEEDBACK_ID) &&   /* 帧 ID 匹配 0x186（后驱电机反馈） */
      (msg->header.DataLength >= FDCAN_DLC_BYTES_5))          /* 数据长度至少 5 字节（确保完整） */
  {
    uint16_t raw_speed;  /* 原始速度值（0~65535 编码） */

    raw_speed = (uint16_t)msg->data[3] |                              /* 速度低字节 (data[3]) */
                (uint16_t)((uint16_t)msg->data[4] << 8U);             /* 速度高字节 (data[4]) 左移 8 位后拼合 */

    g_drive_motor_actual_speed_raw = raw_speed;                        /* 保存原始速度值（调试用） */
    g_drive_motor_actual_speed_mps = DriveMotor_RawFeedbackToSpeed(raw_speed); /* 原始值换算为 m/s */
    g_drive_motor_feedback_last_tick = osKernelGetTickCount();         /* 记录本次反馈时间戳（用于超时诊断） */
    g_drive_motor_feedback_rx_count++;                                 /* 累加反馈帧接收计数 */
  }
  /* EPS 反馈帧 0x18F：解析实际转向角，并解算车辆实际横摆角速度。 */
  else if ((msg->header.Identifier == EPS_FEEDBACK_ID) &&             /* 帧 ID 匹配 0x18F（EPS 转向反馈） */
           (msg->header.DataLength >= FDCAN_DLC_BYTES_3))              /* 数据长度至少 3 字节 */
  {
    float actual_angle_deg;  /* 实际转向角（度） */

    actual_angle_deg = EpsSteer_ParseFeedback(msg->data);              /* 从 CAN 数据字节解析 EPS 实际转向角 */

    g_eps_actual_angle_deg = actual_angle_deg;                         /* 更新全局 EPS 实际角度 */
    g_steer_motor_actual_angle_deg = actual_angle_deg;                 /* 同步更新转向电机实际角度（同一值） */
    g_eps_feedback_last_tick = osKernelGetTickCount();                 /* 记录本次反馈时间戳 */
    g_eps_feedback_rx_count++;                                         /* 累加 EPS 反馈帧接收计数 */

    /* 使用阿克曼模型由实际线速度和实际转向角解算车辆横摆角速度。 */
    g_chassis_actual_angular_z = Chassis_CalcActualYawRate(            /* 阿克曼公式：ω = v/L * tan(δ) */
        g_drive_motor_actual_speed_mps, actual_angle_deg);             /* 输入：实际线速度 (m/s)、实际转向角 (度) */
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) /* FDCAN1 FIFO0 接收中断回调 */
{
  if ((hfdcan == NULL) ||                                         /* 句柄空指针检查 */
      (hfdcan->Instance != FDCAN1) ||                              /* 只处理 FDCAN1 的中断（FDCAN2 不进入此回调） */
      ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) ||     /* 确认是 FIFO0 新消息中断（非满/满/溢出等） */
      (canRxQueueHandle == NULL))                                  /* 队列句柄有效性检查（未初始化则跳过） */
  {
    return;  /* 不满足条件 → 直接返回，不处理 */
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) /* 循环读取 FIFO0 中所有待处理消息 */
  {
    CanRxMsg_t rx_msg;  /* 栈上分配临时 CAN 消息变量 */

    if (HAL_FDCAN_GetRxMessage(hfdcan,               /* 从 FDCAN 外设读取一帧消息 */
                               FDCAN_RX_FIFO0,        /* 指定 FIFO0 */
                               &rx_msg.header,        /* 输出：CAN 帧头（ID、DLC、时间戳） */
                               rx_msg.data) != HAL_OK) /* 输出：CAN 数据字节 */
    {
      break;  /* 读取失败 → 退出循环（防止死循环） */
    }

    (void)osMessageQueuePut(canRxQueueHandle, &rx_msg, 0U, 0U); /* 将 CAN 消息放入队列（非阻塞，队列满则丢弃） */
  }
}


/**
 * @brief       /stm32_ping心跳发布回调。
 * @param       timer         : 触发本回调的 rcl 定时器
 *              last_call_time: 上一次触发时间，当前未使用
 * @retval      无
 * @note        发布成功后ping计数自增；发布失败时累加micro-ROS发布失败计数。
 */
static void ping_timer_callback(rcl_timer_t *timer, int64_t last_call_time) /* 200ms 周期心跳发布回调 */
{
  (void)timer;          /* 消除"未使用参数"编译器警告 */
  (void)last_call_time; /* 消除"未使用参数"编译器警告 */

  /* /stm32_ping使用递增Int32，方便上位机确认链路持续工作。 */
  if (rcl_publish(&g_ping_publisher, &g_ping_msg, NULL) == RCL_RET_OK) /* 发布 Int32 心跳消息（无超时） */
  {
    g_ping_msg.data++;                   /* 发布成功 → 心跳计数器自增 1 */
    g_microros_publish_fail_count = 0U;  /* 发布成功 → 失败计数归零（链路恢复） */
  }
  else
  {
    g_microros_publish_fail_count++;     /* 发布失败 → 累加失败计数（用于诊断） */
    rcl_reset_error();                   /* 清除 rcl 错误状态 */
  }
}

/**
 * @brief       /odom 里程计发布回调 (5Hz)。
 * @param       timer         : 触发本回调的 rcl 定时器
 *              last_call_time: 上一次触发时间，当前未使用
 * @retval      无
 * @note        将实际线速度和横摆角速度积分得到位姿 (x, y, θ)，
 *              按标准 Odometry 消息格式发布，供 Nav2 导航栈使用。
 */
static void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time) /* 5Hz 里程计发布回调 */
{
  float vx = g_drive_motor_actual_speed_mps;  /* 当前实际线速度 (m/s) */
  float vtheta = g_chassis_actual_angular_z;  /* 当前实际横摆角速度 (rad/s) */
  uint32_t now = osKernelGetTickCount();      /* 当前系统 tick 计数 */
  float dt;      /* 时间步长（秒），两次积分之间的时间间隔 */
  float half_theta; /* 半航向角，用于 sin/cos 计算四元数 */

  (void)timer;          /* 消除"未使用参数"编译器警告 */
  (void)last_call_time; /* 消除"未使用参数"编译器警告 */

  /* 计算时间步长 (秒)。 */
  if (g_odom_last_tick == 0U) /* 首次调用，没有上一次时间戳 */
  {
    dt = 0.02f;  /* 首次按 20ms 估算（200ms 周期的保守值，避免 dt=0 导致积分不发散） */
  }
  else
  {
    dt = (float)(now - g_odom_last_tick) * (1.0f / (float)osKernelGetTickFreq()); /* 实际时间差 (tick) ÷ tick 频率 = 秒 */
  }
  g_odom_last_tick = now; /* 保存当前 tick 供下次计算 dt */

  /* 航迹推算: 中值法积分。 */
  g_odom_pose_x += vx * cosf(g_odom_pose_theta + 0.5f * vtheta * dt) * dt;  /* X 方向积分：v·cos(θ+½ωdt)·dt */
  g_odom_pose_y += vx * sinf(g_odom_pose_theta + 0.5f * vtheta * dt) * dt;  /* Y 方向积分：v·sin(θ+½ωdt)·dt */
  g_odom_pose_theta += vtheta * dt;                                          /* 航向角积分：ω·dt */

  /* yaw -> 四元数。 */
  half_theta = 0.5f * g_odom_pose_theta; /* 将航向角减半，用于四元数转换公式 */

  /* 填充标准 Odometry 消息：优先使用 Agent 同步后的ROS时间。 */
  if (rmw_uros_epoch_synchronized()) /* 检查是否已与 Agent 时间同步 */
  {
    int64_t time_ns = rmw_uros_epoch_nanos();                              /* 获取 Agent 同步后的 ROS 时间 (ns) */
    g_odom_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000LL);       /* 秒部分 = 总纳秒 ÷ 10^9 */
    g_odom_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);  /* 纳秒部分 = 总纳秒 mod 10^9 */
  }
  else
  {
    uint32_t tick = osKernelGetTickCount();                                /* 未同步时使用 FreeRTOS tick 作为后备 */
    g_odom_msg.header.stamp.sec = (int32_t)(tick / osKernelGetTickFreq()); /* 秒部分 = tick ÷ 频率 */
    g_odom_msg.header.stamp.nanosec = (uint32_t)((tick % osKernelGetTickFreq()) * (1000000000U / osKernelGetTickFreq())); /* 纳秒部分 = (tick 余数) × (10^9 / 频率) */
  }
  g_odom_msg.pose.pose.position.x = (double)g_odom_pose_x;    /* 位姿 X 坐标（double 类型，ROS 标准） */
  g_odom_msg.pose.pose.position.y = (double)g_odom_pose_y;    /* 位姿 Y 坐标 */
  g_odom_msg.pose.pose.position.z = 0.0;                      /* 位姿 Z 坐标 = 0（平面运动） */
  g_odom_msg.pose.pose.orientation.x = 0.0;                   /* 四元数 x = 0（绕 Z 轴旋转，无 X/Y 分量） */
  g_odom_msg.pose.pose.orientation.y = 0.0;                   /* 四元数 y = 0 */
  g_odom_msg.pose.pose.orientation.z = (double)sinf(half_theta); /* 四元数 z = sin(θ/2) */
  g_odom_msg.pose.pose.orientation.w = (double)cosf(half_theta); /* 四元数 w = cos(θ/2) */
  g_odom_msg.twist.twist.linear.x = (double)vx;               /* 线速度 X 分量 */
  g_odom_msg.twist.twist.linear.y = 0.0;                      /* 线速度 Y 分量 = 0（非完整约束，无侧滑） */
  g_odom_msg.twist.twist.linear.z = 0.0;                      /* 线速度 Z 分量 = 0（平面运动） */
  g_odom_msg.twist.twist.angular.x = 0.0;                     /* 角速度 X 分量 = 0 */
  g_odom_msg.twist.twist.angular.y = 0.0;                     /* 角速度 Y 分量 = 0 */
  g_odom_msg.twist.twist.angular.z = (double)vtheta;          /* 角速度 Z 分量（横摆角速度） */

  if (rcl_publish(&g_odom_publisher, &g_odom_msg, NULL) != RCL_RET_OK) /* 发布 /odom 消息 */
  {
    g_microros_publish_fail_count++;  /* 发布失败 → 累加失败计数 */
    rcl_reset_error();                /* 清除错误状态 */
  }
}

/**
 * @brief       micro-ROS /cmd_vel订阅回调，缓存上位机速度命令。
 * @param       msgin: geometry_msgs/msg/Twist 类型消息指针
 * @retval      无
 * @note        只使用linear.x和angular.z；收到消息时更新接收时间戳和计数，供控制任务做超时保护。
 */
static void cmd_vel_callback(const void *msgin) /* /cmd_vel 订阅回调：由 executor 在新数据到达时调用 */
{
  const geometry_msgs__msg__Twist *cmd_vel = (const geometry_msgs__msg__Twist *)msgin; /* 将 void* 强制转换为 Twist 指针 */

  /* rclc executor收到新/cmd_vel后调用本函数，控制任务随后读取这些全局命令。 */
  if (cmd_vel != NULL)  /* 空指针保护 */
  {
    g_chassis_cmd_linear_x = (float)cmd_vel->linear.x;   /* 提取线速度 X (m/s)，转为 float 存入全局变量 */
    g_chassis_cmd_angular_z = (float)cmd_vel->angular.z;  /* 提取角速度 Z (rad/s)，转为 float 存入全局变量 */
    g_chassis_cmd_last_tick = osKernelGetTickCount();      /* 记录接收时间戳（供 controlTask 超时检测） */
    g_chassis_cmd_rx_count++;                              /* 累加接收计数（诊断用） */
  }
}
/* USER CODE END Application */
