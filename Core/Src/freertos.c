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
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <uxr/client/transport.h>
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

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

static void led_pd10_init(void);
static void led_cmd_callback(const void * msgin);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
/* USER CODE BEGIN StartDefaultTask */

(void) argument;

led_pd10_init();

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
osDelay(1000);
}
}

rcl_allocator_t allocator = rcl_get_default_allocator();

static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t publisher;
static rcl_subscription_t led_subscriber;
static rclc_executor_t executor;
static std_msgs__msg__Int32 msg;
static std_msgs__msg__Bool led_msg;

while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
{
rcl_reset_error();
osDelay(500);
}

while (rclc_node_init_default(&node, "stm32_f407_node", "", &support) != RCL_RET_OK)
{
rcl_reset_error();
osDelay(500);
}

while (rclc_publisher_init_default(
&publisher,
&node,
ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
"/stm32_ping") != RCL_RET_OK)
{
rcl_reset_error();
osDelay(500);
}

while (rclc_subscription_init_default(
&led_subscriber,
&node,
ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
"/led_cmd") != RCL_RET_OK)
{
rcl_reset_error();
osDelay(500);
}

while (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
{
rcl_reset_error();
osDelay(500);
}

msg.data = 0;
led_msg.data = false;

while (rclc_executor_add_subscription(
&executor,
&led_subscriber,
&led_msg,
led_cmd_callback,
ON_NEW_DATA) != RCL_RET_OK)
{
rcl_reset_error();
osDelay(500);
}

uint32_t last_publish = osKernelGetTickCount();

for (;;)
{
rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

if ((spin_ret != RCL_RET_OK) && (spin_ret != RCL_RET_TIMEOUT))
{
rcl_reset_error();
}

uint32_t now = osKernelGetTickCount();

if ((now - last_publish) >= pdMS_TO_TICKS(1000))
{
last_publish = now;

if (rcl_publish(&publisher, &msg, NULL) == RCL_RET_OK)
{
msg.data++;
}
else
{
rcl_reset_error();
}
}

osDelay(10);
}
/* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void led_pd10_init(void)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};

__HAL_RCC_GPIOD_CLK_ENABLE();

HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);

GPIO_InitStruct.Pin = GPIO_PIN_10;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

static void led_cmd_callback(const void * msgin)
{
const std_msgs__msg__Bool * led_cmd = (const std_msgs__msg__Bool *) msgin;

if ((led_cmd != NULL) && led_cmd->data)
{
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
}
else
{
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
}
}
/* USER CODE END Application */

