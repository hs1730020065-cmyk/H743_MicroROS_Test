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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/timer.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <uxr/client/transport.h>
#include "can_motor.h"
#include "can_steer.h"
#include "fdcan.h"
#include "sbus.h"
#include "usart.h"
#include "atk_ms901m.h"
#include "chassis_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
/* FDCAN接收队列元素：保存一帧CAN头和最多8字节数据。 */
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
/* 最近一次后驱电机和转向电机CAN下发状态，供调试或反馈查看。 */
volatile HAL_StatusTypeDef g_can_motor_last_status = HAL_OK;
volatile HAL_StatusTypeDef g_eps_steer_last_status = HAL_OK;
/* SBUS任务解析出的最近一帧遥控数据和控制量。 */
volatile SbusData_t g_sbus_data;
volatile SbusControl_t g_sbus_control;
volatile uint8_t g_sbus_frame_ok = 0U;
volatile uint8_t g_sbus_failsafe = 0U;
volatile uint8_t g_sbus_estop_active = 0U;
/* MS901M初始化结果、最近姿态帧状态和接收计数。 */
volatile uint8_t g_ms901m_init_status = ATK_MS901M_ERROR;
volatile uint8_t g_ms901m_frame_ok = 0U;
volatile uint32_t g_ms901m_rx_count = 0U;
volatile atk_ms901m_attitude_data_t g_ms901m_attitude;
/* /cmd_vel回调写入的最新线速度、角速度和接收时间戳。 */
volatile float g_chassis_cmd_linear_x = 0.0f;
volatile float g_chassis_cmd_angular_z = 0.0f;
volatile uint32_t g_chassis_cmd_rx_count = 0U;
volatile uint32_t g_chassis_cmd_last_tick = 0U;
/* micro-ROS运行诊断计数：agent丢失预留、发布失败和executor spin错误。 */
volatile uint32_t g_microros_agent_lost_count = 0U;
volatile uint32_t g_microros_publish_fail_count = 0U;
volatile uint32_t g_microros_spin_error_count = 0U;
/* micro-ROS发布器和消息对象，由defaultTask初始化后长期使用。 */
static rcl_publisher_t g_ping_publisher;
static rcl_publisher_t g_odom_publisher;
static std_msgs__msg__Int32 g_ping_msg;
static nav_msgs__msg__Odometry g_odom_msg;

/* 里程计位姿积分变量。 */
static float g_odom_pose_x = 0.0f;
static float g_odom_pose_y = 0.0f;
static float g_odom_pose_theta = 0.0f;
static uint32_t g_odom_last_tick = 0U;

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
  .priority = (osPriority_t) osPriorityNormal,
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
static void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
static void cmd_vel_callback(const void *msgin);
static void can_rx_process(const CanRxMsg_t *msg);


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
  canRxQueueHandle = osMessageQueueNew (32, sizeof(CanRxMsg_t), &canRxQueue_attributes);

  /* creation of cmdVelQueue */
  cmdVelQueueHandle = osMessageQueueNew (4, sizeof(uint32_t), &cmdVelQueue_attributes);

  /* creation of sbusQueue */
  sbusQueueHandle = osMessageQueueNew (4, sizeof(uint32_t), &sbusQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 队列已在上方创建，这里没有额外队列。 */
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
  (void) argument;

  /* micro-ROS使用UART1上的自定义传输函数连接agent。 */
  rmw_uros_set_custom_transport(
  true,
  (void *) &huart1,
  cubemx_transport_open,
  cubemx_transport_close,
  cubemx_transport_write,
  cubemx_transport_read);

  /* 将micro-ROS内存分配切换到工程提供的FreeRTOS分配器。 */
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
  static rcl_timer_t odom_timer;
  static rclc_executor_t executor;

  /* 以下micro-ROS对象初始化失败时会清除错误并等待重试。 */
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
  &g_odom_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
  "/odom") != RCL_RET_OK)
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

  /* 里程计消息初始化。 */
  g_odom_msg.header.frame_id.data = (char *)"odom";
  g_odom_msg.header.frame_id.size = 4;
  g_odom_msg.header.frame_id.capacity = 5;
  g_odom_msg.child_frame_id.data = (char *)"base_link";
  g_odom_msg.child_frame_id.size = 9;
  g_odom_msg.child_frame_id.capacity = 10;

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
  &odom_timer,
  &support,
  RCL_MS_TO_NS(200),
  odom_timer_callback) != RCL_RET_OK)
  {
    rcl_reset_error();
    osDelay(500U);
  }

  /* executor容量为3：一个/cmd_vel订阅回调和两个定时器回调。 */
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

  while (rclc_executor_add_timer(&executor, &odom_timer) != RCL_RET_OK)
  {
    rcl_reset_error();
    osDelay(500U);
  }

  /* 通过串口与 Agent 同步时间，使 odom 时间戳与 ROS 时间对齐。 */
  while (rmw_uros_sync_session(1000) != RMW_RET_OK)
  {
    rcl_reset_error();
    osDelay(100U);
  }

  /* 主循环驱动micro-ROS executor，最多等待50ms处理一次事件。 */
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

    /* /cmd_vel超时或从未收到命令时，主动清零目标避免底盘继续运动。 */
    if ((g_chassis_cmd_last_tick == 0U) || ((now - g_chassis_cmd_last_tick) > pdMS_TO_TICKS(500)))
    {
      g_chassis_cmd_linear_x = 0.0f;
      g_chassis_cmd_angular_z = 0.0f;
    }

    /* 将最新线速度和角速度转换为后驱速度、前轮转角，并通过底层接口下发。 */
    Chassis_ControlFromCmdVel(g_chassis_cmd_linear_x, g_chassis_cmd_angular_z);
    /* 保存本周期底层CAN发送状态，便于调试和/chassis_feedback观察。 */
    g_can_motor_last_status = g_drive_motor_can_status;
    g_eps_steer_last_status = g_steer_motor_can_status;
    CanMotor_UpdateDiagnostics();
    osDelay(10U);
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
  (void)argument;

  /* 当前任务尚无具体业务，周期延时保持线程存活。 */
  for(;;)
  {
    CanRxMsg_t rx_msg;

    if (osMessageQueueGet(canRxQueueHandle, &rx_msg, NULL, osWaitForever) == osOK)
    {
      can_rx_process(&rx_msg);
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
  for(;;)
  {
    osDelay(1000U);
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
  (void)argument;

  /* MS901M按115200波特率初始化，初始化结果保存到全局状态。 */
  g_ms901m_init_status = atk_ms901m_init(115200U);

  for (;;)
  {
    atk_ms901m_attitude_data_t attitude;

    /* 每10ms尝试读取一次姿态，单次读取最多等待5ms。 */
    if (atk_ms901m_get_attitude(&attitude, 5U) == ATK_MS901M_EOK)
    {
      /* 读取成功后发布最新姿态，并累加有效帧计数。 */
      g_ms901m_attitude = attitude;
      g_ms901m_frame_ok = 1U;
      g_ms901m_rx_count++;
    }
    else
    {
      /* 本周期没有拿到有效姿态帧，只清状态，不清最后一次姿态值。 */
      g_ms901m_frame_ok = 0U;
    }

    osDelay(10U);
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
  (void)argument;

  /* UART5按SBUS真实接收配置启动，底层会优先使用DMA接收整帧。 */
  SBUS_Init();

  uint32_t last_rx_count = SBUS_GetRxFrameCount();

  for (;;)
  {
    uint32_t rx_count = SBUS_GetRxFrameCount();

    /* 只有帧计数变化时才解析，避免重复处理同一帧SBUS数据。 */
    if (rx_count != last_rx_count)
    {
      SbusData_t data;
      SbusControl_t ctrl;
      const uint8_t *frame = SBUS_GetRxBuffer();

      last_rx_count = rx_count;

      /* 帧头和打包格式检查通过后，将16通道数据转换为控制量。 */
      if (SBUS_TryParseFrame(frame, &data))
      {
        SBUS_ConvertToControl(&data, &ctrl);

        /* 发布最新SBUS原始通道、控制量、failsafe和急停状态。 */
        g_sbus_data = data;
        g_sbus_control = ctrl;
        g_sbus_frame_ok = 1U;
        g_sbus_failsafe = data.failsafe;
        g_sbus_estop_active = ctrl.estop;

      }
      else
      {
        /* 新帧解析失败时仅标记无效，保留上一帧控制状态供上层自行处理。 */
        g_sbus_frame_ok = 0U;
      }
    }

    osDelay(10);
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
  for(;;)
  {
    osDelay(1000U);
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
  for(;;)
  {
    osDelay(1000U);
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
  for(;;)
  {
    osDelay(1000U);
  }
  /* USER CODE END StartLidarTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void can_rx_process(const CanRxMsg_t *msg)
{
  if (msg == NULL)
  {
    return;
  }

  if (msg->header.IdType != FDCAN_STANDARD_ID)
  {
    return;
  }

  /* 后驱电机反馈帧 0x186：解析实际线速度。 */
  if ((msg->header.Identifier == CAN_MOTOR_FEEDBACK_ID) &&
      (msg->header.DataLength >= FDCAN_DLC_BYTES_5))
  {
    uint16_t raw_speed;

    raw_speed = (uint16_t)msg->data[3] |
                (uint16_t)((uint16_t)msg->data[4] << 8U);

    g_drive_motor_actual_speed_raw = raw_speed;
    g_drive_motor_actual_speed_mps = DriveMotor_RawFeedbackToSpeed(raw_speed);
    g_drive_motor_feedback_last_tick = osKernelGetTickCount();
    g_drive_motor_feedback_rx_count++;
  }
  /* EPS 反馈帧 0x18F：解析实际转向角，并解算车辆实际横摆角速度。 */
  else if ((msg->header.Identifier == EPS_FEEDBACK_ID) &&
           (msg->header.DataLength >= FDCAN_DLC_BYTES_3))
  {
    float actual_angle_deg;

    actual_angle_deg = EpsSteer_ParseFeedback(msg->data);

    g_eps_actual_angle_deg = actual_angle_deg;
    g_steer_motor_actual_angle_deg = actual_angle_deg;
    g_eps_feedback_last_tick = osKernelGetTickCount();
    g_eps_feedback_rx_count++;

    /* 使用阿克曼模型由实际线速度和实际转向角解算车辆横摆角速度。 */
    g_chassis_actual_angular_z = Chassis_CalcActualYawRate(
        g_drive_motor_actual_speed_mps, actual_angle_deg);
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((hfdcan == NULL) ||
      (hfdcan->Instance != FDCAN1) ||
      ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) ||
      (canRxQueueHandle == NULL))
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    CanRxMsg_t rx_msg;

    if (HAL_FDCAN_GetRxMessage(hfdcan,
                               FDCAN_RX_FIFO0,
                               &rx_msg.header,
                               rx_msg.data) != HAL_OK)
    {
      break;
    }

    (void)osMessageQueuePut(canRxQueueHandle, &rx_msg, 0U, 0U);
  }
}


/**
 * @brief       /stm32_ping心跳发布回调。
 * @param       timer         : 触发本回调的 rcl 定时器
 *              last_call_time: 上一次触发时间，当前未使用
 * @retval      无
 * @note        发布成功后ping计数自增；发布失败时累加micro-ROS发布失败计数。
 */
static void ping_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)timer;
  (void)last_call_time;

  /* /stm32_ping使用递增Int32，方便上位机确认链路持续工作。 */
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
 * @brief       /odom 里程计发布回调 (5Hz)。
 * @param       timer         : 触发本回调的 rcl 定时器
 *              last_call_time: 上一次触发时间，当前未使用
 * @retval      无
 * @note        将实际线速度和横摆角速度积分得到位姿 (x, y, θ)，
 *              按标准 Odometry 消息格式发布，供 Nav2 导航栈使用。
 */
static void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  float vx = g_drive_motor_actual_speed_mps;
  float vtheta = g_chassis_actual_angular_z;
  uint32_t now = osKernelGetTickCount();
  float dt;
  float half_theta;

  (void)timer;
  (void)last_call_time;

  /* 计算时间步长 (秒)。 */
  if (g_odom_last_tick == 0U)
  {
    dt = 0.02f;  /* 首次按 20ms 估算。 */
  }
  else
  {
    dt = (float)(now - g_odom_last_tick) * (1.0f / (float)osKernelGetTickFreq());
  }
  g_odom_last_tick = now;

  /* 航迹推算: 中值法积分。 */
  g_odom_pose_x += vx * cosf(g_odom_pose_theta + 0.5f * vtheta * dt) * dt;
  g_odom_pose_y += vx * sinf(g_odom_pose_theta + 0.5f * vtheta * dt) * dt;
  g_odom_pose_theta += vtheta * dt;

  /* yaw -> 四元数。 */
  half_theta = 0.5f * g_odom_pose_theta;

  /* 填充标准 Odometry 消息：优先使用 Agent 同步后的ROS时间。 */
  if (rmw_uros_epoch_synchronized())
  {
    int64_t time_ns = rmw_uros_epoch_nanos();
    g_odom_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000LL);
    g_odom_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);
  }
  else
  {
    uint32_t tick = osKernelGetTickCount();
    g_odom_msg.header.stamp.sec = (int32_t)(tick / osKernelGetTickFreq());
    g_odom_msg.header.stamp.nanosec = (uint32_t)((tick % osKernelGetTickFreq()) * (1000000000U / osKernelGetTickFreq()));
  }
  g_odom_msg.pose.pose.position.x = (double)g_odom_pose_x;
  g_odom_msg.pose.pose.position.y = (double)g_odom_pose_y;
  g_odom_msg.pose.pose.position.z = 0.0;
  g_odom_msg.pose.pose.orientation.x = 0.0;
  g_odom_msg.pose.pose.orientation.y = 0.0;
  g_odom_msg.pose.pose.orientation.z = (double)sinf(half_theta);
  g_odom_msg.pose.pose.orientation.w = (double)cosf(half_theta);
  g_odom_msg.twist.twist.linear.x = (double)vx;
  g_odom_msg.twist.twist.linear.y = 0.0;
  g_odom_msg.twist.twist.linear.z = 0.0;
  g_odom_msg.twist.twist.angular.x = 0.0;
  g_odom_msg.twist.twist.angular.y = 0.0;
  g_odom_msg.twist.twist.angular.z = (double)vtheta;

  if (rcl_publish(&g_odom_publisher, &g_odom_msg, NULL) != RCL_RET_OK)
  {
    g_microros_publish_fail_count++;
    rcl_reset_error();
  }
}

/**
 * @brief       micro-ROS /cmd_vel订阅回调，缓存上位机速度命令。
 * @param       msgin: geometry_msgs/msg/Twist 类型消息指针
 * @retval      无
 * @note        只使用linear.x和angular.z；收到消息时更新接收时间戳和计数，供控制任务做超时保护。
 */
static void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *cmd_vel = (const geometry_msgs__msg__Twist *)msgin;

  /* rclc executor收到新/cmd_vel后调用本函数，控制任务随后读取这些全局命令。 */
  if (cmd_vel != NULL)
  {
    g_chassis_cmd_linear_x = (float)cmd_vel->linear.x;
    g_chassis_cmd_angular_z = (float)cmd_vel->angular.z;
    g_chassis_cmd_last_tick = osKernelGetTickCount();
    g_chassis_cmd_rx_count++;
  }
}
/* USER CODE END Application */
