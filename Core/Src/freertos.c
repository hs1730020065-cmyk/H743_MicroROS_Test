/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/timer.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <uxr/client/transport.h>
#include "can_motor.h"
#include "can_steer.h"
#include "sbus.h"
#include "usart.h"
#include "atk_ms901m.h"
#include "chassis_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
typedef struct
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[8];
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
volatile HAL_StatusTypeDef g_can_motor_last_status = HAL_OK;
volatile HAL_StatusTypeDef g_eps_steer_last_status = HAL_OK;
volatile SbusData_t g_sbus_data;
volatile SbusControl_t g_sbus_control;
volatile uint8_t g_sbus_frame_ok = 0U;
volatile uint8_t g_sbus_failsafe = 0U;
volatile uint8_t g_sbus_estop_active = 0U;
volatile uint8_t g_ms901m_init_status = ATK_MS901M_ERROR;
volatile uint8_t g_ms901m_frame_ok = 0U;
volatile uint32_t g_ms901m_rx_count = 0U;
volatile atk_ms901m_attitude_data_t g_ms901m_attitude;
volatile float g_chassis_cmd_linear_x = 0.0f;
volatile float g_chassis_cmd_angular_z = 0.0f;
volatile uint32_t g_chassis_cmd_rx_count = 0U;
volatile uint32_t g_chassis_cmd_last_tick = 0U;
volatile uint32_t g_microros_agent_lost_count = 0U;
volatile uint32_t g_microros_publish_fail_count = 0U;
volatile uint32_t g_microros_spin_error_count = 0U;
static rcl_publisher_t g_ping_publisher;
static rcl_publisher_t g_chassis_feedback_publisher;
static std_msgs__msg__Int32 g_ping_msg;
static geometry_msgs__msg__Twist g_chassis_feedback_msg;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
uint32_t controlTaskBuffer[ 1024 ];
osStaticThreadDef_t controlTaskControlBlock;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .cb_mem = &controlTaskControlBlock,
  .cb_size = sizeof(controlTaskControlBlock),
  .stack_mem = &controlTaskBuffer[0],
  .stack_size = sizeof(controlTaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for canTask */
osThreadId_t canTaskHandle;
uint32_t canTaskBuffer[ 1024 ];
osStaticThreadDef_t canTaskControlBlock;
const osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .cb_mem = &canTaskControlBlock,
  .cb_size = sizeof(canTaskControlBlock),
  .stack_mem = &canTaskBuffer[0],
  .stack_size = sizeof(canTaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for encoderTask */
osThreadId_t encoderTaskHandle;
uint32_t encoderTaskBuffer[ 512 ];
osStaticThreadDef_t encoderTaskControlBlock;
const osThreadAttr_t encoderTask_attributes = {
  .name = "encoderTask",
  .cb_mem = &encoderTaskControlBlock,
  .cb_size = sizeof(encoderTaskControlBlock),
  .stack_mem = &encoderTaskBuffer[0],
  .stack_size = sizeof(encoderTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
uint32_t imuTaskBuffer[ 1024 ];
osStaticThreadDef_t imuTaskControlBlock;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .cb_mem = &imuTaskControlBlock,
  .cb_size = sizeof(imuTaskControlBlock),
  .stack_mem = &imuTaskBuffer[0],
  .stack_size = sizeof(imuTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sbusTask */
osThreadId_t sbusTaskHandle;
uint32_t sbusTaskBuffer[ 1024 ];
osStaticThreadDef_t sbusTaskControlBlock;
const osThreadAttr_t sbusTask_attributes = {
  .name = "sbusTask",
  .cb_mem = &sbusTaskControlBlock,
  .cb_size = sizeof(sbusTaskControlBlock),
  .stack_mem = &sbusTaskBuffer[0],
  .stack_size = sizeof(sbusTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for batteryTask */
osThreadId_t batteryTaskHandle;
uint32_t batteryTaskBuffer[ 512 ];
osStaticThreadDef_t batteryTaskControlBlock;
const osThreadAttr_t batteryTask_attributes = {
  .name = "batteryTask",
  .cb_mem = &batteryTaskControlBlock,
  .cb_size = sizeof(batteryTaskControlBlock),
  .stack_mem = &batteryTaskBuffer[0],
  .stack_size = sizeof(batteryTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;
uint32_t gpsTaskBuffer[ 1024 ];
osStaticThreadDef_t gpsTaskControlBlock;
const osThreadAttr_t gpsTask_attributes = {
  .name = "gpsTask",
  .cb_mem = &gpsTaskControlBlock,
  .cb_size = sizeof(gpsTaskControlBlock),
  .stack_mem = &gpsTaskBuffer[0],
  .stack_size = sizeof(gpsTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for lidarTask */
osThreadId_t lidarTaskHandle;
uint32_t lidarTaskBuffer[ 1024 ];
osStaticThreadDef_t lidarTaskControlBlock;
const osThreadAttr_t lidarTask_attributes = {
  .name = "lidarTask",
  .cb_mem = &lidarTaskControlBlock,
  .cb_size = sizeof(lidarTaskControlBlock),
  .stack_mem = &lidarTaskBuffer[0],
  .stack_size = sizeof(lidarTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for canRxQueue */
osMessageQueueId_t canRxQueueHandle;
uint8_t canRxQueueBuffer[ 32 * sizeof( CanRxMsg_t ) ];
osStaticMessageQDef_t canRxQueueControlBlock;
const osMessageQueueAttr_t canRxQueue_attributes = {
  .name = "canRxQueue",
  .cb_mem = &canRxQueueControlBlock,
  .cb_size = sizeof(canRxQueueControlBlock),
  .mq_mem = &canRxQueueBuffer,
  .mq_size = sizeof(canRxQueueBuffer)
};
/* Definitions for cmdVelQueue */
osMessageQueueId_t cmdVelQueueHandle;
uint8_t cmdVelQueueBuffer[ 4 * sizeof( uint32_t ) ];
osStaticMessageQDef_t cmdVelQueueControlBlock;
const osMessageQueueAttr_t cmdVelQueue_attributes = {
  .name = "cmdVelQueue",
  .cb_mem = &cmdVelQueueControlBlock,
  .cb_size = sizeof(cmdVelQueueControlBlock),
  .mq_mem = &cmdVelQueueBuffer,
  .mq_size = sizeof(cmdVelQueueBuffer)
};
/* Definitions for sbusQueue */
osMessageQueueId_t sbusQueueHandle;
uint8_t sbusQueueBuffer[ 4 * sizeof( uint32_t ) ];
osStaticMessageQDef_t sbusQueueControlBlock;
const osMessageQueueAttr_t sbusQueue_attributes = {
  .name = "sbusQueue",
  .cb_mem = &sbusQueueControlBlock,
  .cb_size = sizeof(sbusQueueControlBlock),
  .mq_mem = &sbusQueueBuffer,
  .mq_size = sizeof(sbusQueueBuffer)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);

size_t cubemx_transport_write(
struct uxrCustomTransport * transport,
const uint8_t * buf,
size_t len,
uint8_t * err);

size_t cubemx_transport_read(
struct uxrCustomTransport * transport,
uint8_t * buf,
size_t len,
int timeout,
uint8_t * err);
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);

size_t cubemx_transport_write(
struct uxrCustomTransport * transport,
const uint8_t * buf,
size_t len,
uint8_t * err);

size_t cubemx_transport_read(
struct uxrCustomTransport * transport,
uint8_t * buf,
size_t len,
int timeout,
uint8_t * err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

static void ping_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
static void chassis_feedback_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
static void cmd_vel_callback(const void *msgin);


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartControlTask(void *argument);
void StartCanTask(void *argument);
void StartEncoderTask(void *argument);
void StartImuTask(void *argument);
void StartSbusTask(void *argument);
void StartBatteryTask(void *argument);
void StartGpsTask(void *argument);
void StartLidarTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canRxQueue */
  canRxQueueHandle = osMessageQueueNew (32, sizeof(CanRxMsg_t), &canRxQueue_attributes);

  /* creation of cmdVelQueue */
  cmdVelQueueHandle = osMessageQueueNew (4, sizeof(uint32_t), &cmdVelQueue_attributes);

  /* creation of sbusQueue */
  sbusQueueHandle = osMessageQueueNew (4, sizeof(uint32_t), &sbusQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(StartControlTask, NULL, &controlTask_attributes);

  /* creation of canTask */
  canTaskHandle = osThreadNew(StartCanTask, NULL, &canTask_attributes);

  /* creation of encoderTask */
  encoderTaskHandle = osThreadNew(StartEncoderTask, NULL, &encoderTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);

  /* creation of sbusTask */
  sbusTaskHandle = osThreadNew(StartSbusTask, NULL, &sbusTask_attributes);

  /* creation of batteryTask */
  batteryTaskHandle = osThreadNew(StartBatteryTask, NULL, &batteryTask_attributes);

  /* creation of gpsTask */
  gpsTaskHandle = osThreadNew(StartGpsTask, NULL, &gpsTask_attributes);

  /* creation of lidarTask */
  lidarTaskHandle = osThreadNew(StartLidarTask, NULL, &lidarTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief       micro-ROS 默认通信任务
  * @param       argument: 任务参数，当前未使用
  * @retval      无
  * @note        诊断固件：该任务只初始化 micro-ROS 节点，
  *              并周期发布 /stm32_ping，用于验证串口和 agent 链路。
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void) argument;

  rmw_uros_set_custom_transport(
  true,
  (void *) &huart1,
  cubemx_transport_open,
  cubemx_transport_close,
  cubemx_transport_write,
  cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();

  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator))
  {
    for (;;)
    {
      osDelay(1000U);
    }
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();

  static rclc_support_t support;
  static rcl_node_t node;
  static rcl_subscription_t cmd_vel_subscriber;
  static geometry_msgs__msg__Twist cmd_vel_msg;
  static rcl_timer_t ping_timer;
  static rcl_timer_t chassis_feedback_timer;
  static rclc_executor_t executor;

  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
  {
    rcl_reset_error();
    osDelay(500U);
  }

  while (rclc_node_init_default(&node, "stm32_h743_node", "", &support) != RCL_RET_OK)
  {
    rcl_reset_error();
    osDelay(500U);
  }

  while (rclc_publisher_init_default(
  &g_ping_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  "/stm32_ping") != RCL_RET_OK)
  {
    rcl_reset_error();
    osDelay(500U);
  }

  while (rclc_publisher_init_default(
  &g_chassis_feedback_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  "/chassis_feedback") != RCL_RET_OK)
  {
    rcl_reset_error();
    osDelay(500U);
  }

  while (rclc_subscription_init_default(
  &cmd_vel_subscriber,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  "/cmd_vel") != RCL_RET_OK)
  {
    rcl_reset_error();
    osDelay(500U);
  }

  g_ping_msg.data = 0;
  cmd_vel_msg.linear.x = 0.0;
  cmd_vel_msg.linear.y = 0.0;
  cmd_vel_msg.linear.z = 0.0;
  cmd_vel_msg.angular.x = 0.0;
  cmd_vel_msg.angular.y = 0.0;
  cmd_vel_msg.angular.z = 0.0;
  g_chassis_feedback_msg = cmd_vel_msg;

  while (rclc_timer_init_default(
  &ping_timer,
  &support,
  RCL_MS_TO_NS(200),
  ping_timer_callback) != RCL_RET_OK)
  {
    rcl_reset_error();
    osDelay(500U);
  }

  while (rclc_timer_init_default(
  &chassis_feedback_timer,
  &support,
  RCL_MS_TO_NS(200),
  chassis_feedback_timer_callback) != RCL_RET_OK)
  {
    rcl_reset_error();
    osDelay(500U);
  }

  while (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK)
  {
    rcl_reset_error();
    osDelay(500U);
  }

  while (rclc_executor_add_subscription(
  &executor,
  &cmd_vel_subscriber,
  &cmd_vel_msg,
  cmd_vel_callback,
  ON_NEW_DATA) != RCL_RET_OK)
  {
    rcl_reset_error();
    osDelay(500U);
  }

  while (rclc_executor_add_timer(&executor, &ping_timer) != RCL_RET_OK)
  {
    rcl_reset_error();
    osDelay(500U);
  }

  while (rclc_executor_add_timer(&executor, &chassis_feedback_timer) != RCL_RET_OK)
  {
    rcl_reset_error();
    osDelay(500U);
  }

  for (;;)
  {
    rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));

    if ((spin_ret != RCL_RET_OK) && (spin_ret != RCL_RET_TIMEOUT))
    {
      g_microros_spin_error_count++;
      rcl_reset_error();
    }

    osDelay(10U);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief       底盘 10ms 控制任务
* @param       argument: 任务参数，当前未使用
* @retval      无
* @note        周期读取 /cmd_vel 更新的线速度和角速度，
*              调用 Chassis_ControlFromCmdVel() 计算后驱速度和前轮转角。
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  (void)argument;

  /*
   * 控制任务启动时先打开 FDCAN1，并把后驱和转向都置到安全零值。
   * 后续 10ms 循环只负责根据 /cmd_vel 周期刷新目标。
   */
  CanMotor_Init();
  g_can_motor_last_status = CanMotor_Stop();
  g_eps_steer_last_status = EpsSteer_SetManualMode();
  g_eps_steer_last_status = EpsSteer_SetZero();

  /*
   * 10ms 底盘控制循环。
   * micro-ROS 的 /cmd_vel 回调负责更新 g_chassis_cmd_linear_x 和 g_chassis_cmd_angular_z，
   * 这里统一转换成后驱电机目标速度和前轮转向目标角度。
   */
  for (;;)
  {
    uint32_t now = osKernelGetTickCount();

    if ((g_chassis_cmd_last_tick == 0U) || ((now - g_chassis_cmd_last_tick) > pdMS_TO_TICKS(500)))
    {
      g_chassis_cmd_linear_x = 0.0f;
      g_chassis_cmd_angular_z = 0.0f;
    }

    Chassis_ControlFromCmdVel(g_chassis_cmd_linear_x, g_chassis_cmd_angular_z);
    g_can_motor_last_status = g_drive_motor_can_status;
    g_eps_steer_last_status = g_steer_motor_can_status;
    CanMotor_UpdateDiagnostics();
    osDelay(10U);
  }

  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartCanTask */
/**
* @brief Function implementing the canTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTask */
void StartCanTask(void *argument)
{
  /* USER CODE BEGIN StartCanTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000U);
  }
  /* USER CODE END StartCanTask */
}

/* USER CODE BEGIN Header_StartEncoderTask */
/**
* @brief Function implementing the encoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEncoderTask */
void StartEncoderTask(void *argument)
{
  /* USER CODE BEGIN StartEncoderTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000U);
  }
  /* USER CODE END StartEncoderTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
  (void)argument;

  g_ms901m_init_status = atk_ms901m_init(115200U);

  for (;;)
  {
    atk_ms901m_attitude_data_t attitude;

    if (atk_ms901m_get_attitude(&attitude, 5U) == ATK_MS901M_EOK)
    {
      g_ms901m_attitude = attitude;
      g_ms901m_frame_ok = 1U;
      g_ms901m_rx_count++;
    }
    else
    {
      g_ms901m_frame_ok = 0U;
    }

    osDelay(10U);
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartSbusTask */
/**
* @brief Function implementing the sbusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSbusTask */
void StartSbusTask(void *argument)
{
  /* USER CODE BEGIN StartSbusTask */
  (void)argument;

  SBUS_Init();

  uint32_t last_rx_count = SBUS_GetRxFrameCount();

  for (;;)
  {
    uint32_t rx_count = SBUS_GetRxFrameCount();

    if (rx_count != last_rx_count)
    {
      SbusData_t data;
      SbusControl_t ctrl;
      const uint8_t *frame = SBUS_GetRxBuffer();

      last_rx_count = rx_count;

      if (SBUS_TryParseFrame(frame, &data))
      {
        SBUS_ConvertToControl(&data, &ctrl);

        g_sbus_data = data;
        g_sbus_control = ctrl;
        g_sbus_frame_ok = 1U;
        g_sbus_failsafe = data.failsafe;
        g_sbus_estop_active = ctrl.estop;

      }
      else
      {
        g_sbus_frame_ok = 0U;
      }
    }

    osDelay(10);
  }

  /* USER CODE END StartSbusTask */
}

/* USER CODE BEGIN Header_StartBatteryTask */
/**
* @brief Function implementing the batteryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBatteryTask */
void StartBatteryTask(void *argument)
{
  /* USER CODE BEGIN StartBatteryTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000U);
  }
  /* USER CODE END StartBatteryTask */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
* @brief Function implementing the gpsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
  /* USER CODE BEGIN StartGpsTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000U);
  }
  /* USER CODE END StartGpsTask */
}

/* USER CODE BEGIN Header_StartLidarTask */
/**
* @brief Function implementing the lidarTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLidarTask */
void StartLidarTask(void *argument)
{
  /* USER CODE BEGIN StartLidarTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000U);
  }
  /* USER CODE END StartLidarTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/**
 * @brief       /stm32_ping 定时发布回调函数
 * @param       timer         : 触发本回调的 rcl 定时器
 *              last_call_time: 上一次触发时间，当前未使用
 * @retval      无
 */
static void ping_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)timer;
  (void)last_call_time;

  if (rcl_publish(&g_ping_publisher, &g_ping_msg, NULL) == RCL_RET_OK)
  {
    g_ping_msg.data++;
    g_microros_publish_fail_count = 0U;
  }
  else
  {
    g_microros_publish_fail_count++;
    rcl_reset_error();
  }
}

/**
 * @brief       /chassis_feedback 定时发布回调函数
 * @param       timer         : 触发本回调的 rcl 定时器
 *              last_call_time: 上一次触发时间，当前未使用
 * @retval      无
 */
static void chassis_feedback_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)timer;
  (void)last_call_time;

  g_chassis_feedback_msg.linear.x = (double)g_chassis_cmd_linear_x;
  g_chassis_feedback_msg.linear.y = (double)g_drive_motor_target_speed_mps;
  g_chassis_feedback_msg.linear.z = (double)g_fdcan_tx_fifo_free_level;
  g_chassis_feedback_msg.angular.x = (double)g_fdcan_tx_error_count;
  g_chassis_feedback_msg.angular.y = (double)g_steer_motor_target_angle_deg;
  g_chassis_feedback_msg.angular.z = (double)g_chassis_cmd_angular_z;

  if (rcl_publish(&g_chassis_feedback_publisher, &g_chassis_feedback_msg, NULL) != RCL_RET_OK)
  {
    g_microros_publish_fail_count++;
    rcl_reset_error();
  }
}

/**
 * @brief       micro-ROS /cmd_vel 订阅回调函数
 * @param       msgin: geometry_msgs/msg/Twist 类型消息指针
 * @retval      无
 * @note        只使用 linear.x 和 angular.z，分别作为阿克曼底盘线速度和角速度输入。
 */
static void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *cmd_vel = (const geometry_msgs__msg__Twist *)msgin;

  if (cmd_vel != NULL)
  {
    g_chassis_cmd_linear_x = (float)cmd_vel->linear.x;
    g_chassis_cmd_angular_z = (float)cmd_vel->angular.z;
    g_chassis_cmd_last_tick = osKernelGetTickCount();
    g_chassis_cmd_rx_count++;
  }
}
/* USER CODE END Application */