# FreeRTOS 运行时间表

本文档整理当前工程里 FreeRTOS 的实际运行安排，重点回答“每个任务什么时候跑、多久跑一次、谁会抢占谁、数据从中断到任务怎么流动”。

## 1. 调度基础

当前工程的调度基础如下：

| 项目 | 当前值 | 含义 |
| --- | --- | --- |
| RTOS tick | 1000 Hz | 1 tick = 1 ms |
| 调度方式 | 抢占式 | 高优先级任务 ready 后可抢占低优先级任务 |
| 时间基准 | FreeRTOS tick | `osDelay(10U)` 约等于让出 CPU 10 ms |
| HAL 时间基准 | TIM6 | HAL 的 `uwTick` 由 TIM6 以 1 ms 中断递增 |
| 最高可调用 RTOS API 的中断优先级边界 | 5 | 优先级数值 >= 5 的中断可以调用 ISR 安全的 RTOS API |
| FreeRTOS 软件定时器 | 已启用 | 但当前业务没有创建 CMSIS-OS timer，micro-ROS 使用自己的 rcl timer |

注意：下面的表是“调度意图表”。实际运行相位会受启动初始化、CAN/串口中断、micro-ROS agent 是否在线、任务执行耗时影响，所以不能理解成硬实时的固定相位波形。

## 2. 任务周期总表

调度器启动后，当前工程一共创建 9 个业务任务。

| 任务名 | 函数 | 优先级 | 栈大小 | 触发方式 | 周期/等待 | 主要工作 | 备注 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `controlTask` | `StartControlTask` | AboveNormal | 4096 B | 固定周期 | 10 ms | 底盘控制，读取 `/cmd_vel` 缓存，计算后驱速度和 EPS 转角，下发 CAN | 当前最高业务优先级之一 |
| `canTask` | `StartCanTask` | AboveNormal | 4096 B | 队列事件 | `osWaitForever` | 从 `canRxQueue` 取 CAN 反馈帧并解析 | 没有帧时阻塞，不占 CPU |
| `defaultTask` | `StartDefaultTask` | Normal | 12000 B | 循环 + rcl timer | spin 最多 50 ms，然后 delay 10 ms | micro-ROS 节点、`/cmd_vel` 订阅、`/stm32_ping` 和 `/odom` 发布 | agent 不在线时会在初始化阶段反复 500 ms 重试 |
| `imuTask` | `StartImuTask` | Normal | 4096 B | 固定周期 | 10 ms | 读取 MS901M 姿态，单次读取最多等待 5 ms | USART2 中断把字节放入 FIFO，任务解析 |
| `sbusTask` | `StartSbusTask` | Normal | 4096 B | 固定轮询 | 10 ms | 检查 SBUS 帧计数，发现新帧后解析 16 通道和控制量 | 目前只更新状态变量，还没有接管底盘控制 |
| `encoderTask` | `StartEncoderTask` | Normal | 2048 B | 占位周期 | 1000 ms | 暂无实际采集逻辑 | 只 delay 保持任务存在 |
| `batteryTask` | `StartBatteryTask` | Low | 2048 B | 占位周期 | 1000 ms | 暂无电池采样逻辑 | ADC 已初始化，但任务未启动采样处理 |
| `gpsTask` | `StartGpsTask` | Low | 4096 B | 占位周期 | 1000 ms | 暂无定位读取逻辑 | 只 delay |
| `lidarTask` | `StartLidarTask` | Low | 4096 B | 占位周期 | 1000 ms | 暂无雷达读取逻辑 | 只 delay |

## 3. 200 ms 超周期运行表

以 200 ms 作为一个观察窗口，当前主要周期任务大致这样运行：

| 时间点 | 必跑/可能运行的任务 | 说明 |
| --- | --- | --- |
| 0 ms | `controlTask`、`imuTask`、`sbusTask`、`defaultTask` | 10 ms 任务各执行一次。`defaultTask` 可能处理 micro-ROS 事件。 |
| 10 ms | `controlTask`、`imuTask`、`sbusTask` | 底盘控制、IMU 读取、SBUS 检查再次执行。 |
| 20 ms | `controlTask`、`imuTask`、`sbusTask` | 同上。 |
| 30 ms 到 190 ms | `controlTask`、`imuTask`、`sbusTask` 每 10 ms 重复 | 高优先级的 `controlTask` 先运行；同优先级/低优先级任务在它 delay 后运行。 |
| 任意时刻 | `canTask` | FDCAN 中断收到反馈帧后写入 `canRxQueue`，`canTask` 立即被唤醒解析。 |
| 任意时刻 | USART/DMA/TIM6/FDCAN 中断 | 中断优先于任务执行。短中断处理完后回到 RTOS 调度。 |
| 约 200 ms | `defaultTask` 内的 rcl timer | `/stm32_ping` 发布一次，`/odom` 发布一次。具体相位取决于 micro-ROS 初始化完成的时间。 |

简化成时间轴就是：

```text
1 ms tick:    | | | | | | | | | | ...
controlTask: 0ms 10ms 20ms 30ms ... 190ms 200ms ...
imuTask:     0ms 10ms 20ms 30ms ... 190ms 200ms ...
sbusTask:    0ms 10ms 20ms 30ms ... 190ms 200ms ...
defaultTask: spin_some(<=50ms) -> delay 10ms -> repeat
rcl timers:  /stm32_ping 200ms, /odom 200ms
canTask:     CAN frame arrives -> queue wakeup -> parse -> block again
```

## 4. 启动后的第一次调度

`main()` 先初始化外设，再初始化 RTOS 对象，最后 `osKernelStart()` 开始调度。

```text
main()
  -> HAL/Clock/GPIO/DMA/UART/FDCAN/TIM/ADC/I2C 初始化
  -> osKernelInitialize()
  -> MX_FREERTOS_Init()
       -> 创建 canRxQueue/cmdVelQueue/sbusQueue
       -> 创建 9 个任务
  -> osKernelStart()
```

调度器刚启动时，`controlTask` 和 `canTask` 是最高业务优先级：

1. `canTask` 运行后马上阻塞在 `canRxQueue`，等待 CAN 帧。
2. `controlTask` 先执行 `CanMotor_Init()`、`CanMotor_Stop()`、`EpsSteer_SetManualMode()`、`EpsSteer_SetZero()`，然后进入 10 ms 控制循环。
3. Normal 优先级任务开始运行：`defaultTask` 初始化 micro-ROS，`imuTask` 初始化 MS901M，`sbusTask` 初始化 SBUS，`encoderTask` 进入 1000 ms delay。
4. Low 优先级的 `batteryTask/gpsTask/lidarTask` 进入 1000 ms delay。

## 5. 核心任务细节

### `controlTask`, 10 ms

`controlTask` 是底盘实际下发任务。每 10 ms 执行一次：

```text
读取当前 tick
检查 /cmd_vel 是否超过 500 ms 未更新
  -> 如果超时，线速度和角速度清零
Chassis_ControlFromCmdVel(linear_x, angular_z)
  -> Ackermann 计算
  -> 后驱电机速度 CAN 下发
  -> EPS 转角 CAN 下发
更新 FDCAN 诊断变量
osDelay(10)
```

关键结论：

| 项目 | 当前值 |
| --- | --- |
| 控制周期 | 10 ms |
| 控制频率 | 100 Hz |
| 命令超时保护 | 500 ms |
| 输入命令 | `g_chassis_cmd_linear_x`、`g_chassis_cmd_angular_z` |
| 输出执行 | 后驱电机 CAN + EPS CAN |

### `canTask`, 事件触发

`canTask` 没有固定周期。它平时一直阻塞：

```c
osMessageQueueGet(canRxQueueHandle, &rx_msg, NULL, osWaitForever)
```

FDCAN 收到新帧后的路径是：

```text
FDCAN1 收到 FIFO0 新消息
  -> FDCAN1_IT0_IRQHandler()
  -> HAL_FDCAN_IRQHandler()
  -> HAL_FDCAN_RxFifo0Callback()
  -> osMessageQueuePut(canRxQueue)
  -> canTask 被唤醒
  -> can_rx_process()
```

当前解析的反馈帧：

| CAN ID | 来源 | 解析结果 |
| --- | --- | --- |
| `0x186` | 后驱电机反馈 | 实际速度 raw 和 m/s |
| `0x18F` | EPS 反馈 | 实际转角，并换算车辆实际 yaw rate |

### `defaultTask`, micro-ROS

`defaultTask` 是 micro-ROS 主任务。初始化完成后循环：

```c
rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
osDelay(10U);
```

它内部有 3 个 executor handle：

| 类型 | 名称 | 周期/触发 |
| --- | --- | --- |
| subscription | `/cmd_vel` | 收到新消息时触发 |
| timer | `/stm32_ping` | 200 ms |
| timer | `/odom` | 200 ms |

`/cmd_vel` 到底盘控制的数据链路：

```text
ROS2 /cmd_vel
  -> defaultTask 的 rclc executor
  -> cmd_vel_callback()
  -> 更新 g_chassis_cmd_linear_x / g_chassis_cmd_angular_z / g_chassis_cmd_last_tick
  -> controlTask 在下一个 10 ms 周期读取
  -> CAN 下发后驱和转向目标
```

`/odom` 发布频率是 5 Hz。它读取：

```text
g_drive_motor_actual_speed_mps
g_chassis_actual_angular_z
```

然后积分更新：

```text
g_odom_pose_x
g_odom_pose_y
g_odom_pose_theta
```

注意：代码里 `/odom` 第一次回调在 `g_odom_last_tick == 0` 时使用 `dt = 0.02f`，也就是 20 ms；之后才按 tick 差计算真实 dt。由于 rcl timer 实际是 200 ms，首帧里程计会有一个小的首帧时间估算误差。

### `imuTask`, 10 ms

启动时：

```text
atk_ms901m_init(115200)
```

循环中每 10 ms：

```text
atk_ms901m_get_attitude(&attitude, 5)
成功 -> 更新 g_ms901m_attitude / g_ms901m_frame_ok / g_ms901m_rx_count
失败 -> g_ms901m_frame_ok = 0
osDelay(10)
```

USART2 接收中断不直接解析姿态，只把字节写入 MS901M FIFO，`imuTask` 再从 FIFO 里取完整帧。

### `sbusTask`, 10 ms

启动时：

```text
SBUS_Init()
```

循环中每 10 ms：

```text
读取 SBUS_GetRxFrameCount()
如果帧计数变化：
  -> SBUS_GetRxBuffer()
  -> SBUS_TryParseFrame()
  -> SBUS_ConvertToControl()
  -> 更新 g_sbus_data / g_sbus_control / failsafe / estop
osDelay(10)
```

当前 SBUS 只负责解析和更新全局状态，还没有进入 `controlTask` 的仲裁逻辑。所以现在真正驱动底盘的是 `/cmd_vel`。

## 6. 中断时间表

中断不是 FreeRTOS 任务，但它们会打断任务运行。当前关键中断如下：

| 中断 | NVIC 优先级 | 触发源 | 当前作用 |
| --- | --- | --- | --- |
| `TIM6_DAC_IRQn` | 15 | TIM6 1 ms | HAL `uwTick` 时间基准 |
| `FDCAN1_IT0_IRQn` | 5 | FDCAN FIFO0 新消息 | 把 CAN 帧放入 `canRxQueue` |
| `USART1_IRQn` | 5 | micro-ROS 串口 | HAL UART 中断处理，配合 micro-ROS transport |
| `USART2_IRQn` | 5 | MS901M 串口 | 调用 `ATK_MS901M_UART_IRQHandler()`，字节入 FIFO |
| `UART5_IRQn` | 5 | SBUS 串口 | HAL UART 回调，更新 SBUS 接收帧 |
| `USART3_IRQn` | 6 | USART3 | HAL UART 中断处理 |
| `UART4_IRQn` | 6 | UART4 | HAL UART 中断处理 |
| `DMA1_Stream0..6_IRQn` | 5 | USART/ADC/UART DMA | HAL DMA 中断处理 |

最关键的设计点是：FDCAN 中断只搬运数据到队列，真正解析放到 `canTask`。这样中断占用时间短，底盘控制任务也更稳定。

## 7. 队列和共享变量

当前创建了 3 个 CMSIS-RTOS2 队列：

| 队列名 | 容量 | 元素类型 | 当前状态 |
| --- | --- | --- | --- |
| `canRxQueue` | 32 | `CanRxMsg_t` | 正在使用，FDCAN 中断写入，`canTask` 读取 |
| `cmdVelQueue` | 4 | `uint32_t` | 已创建但未使用 |
| `sbusQueue` | 4 | `uint32_t` | 已创建但未使用 |

当前 `/cmd_vel` 没有走队列，而是直接更新全局变量：

```c
g_chassis_cmd_linear_x
g_chassis_cmd_angular_z
g_chassis_cmd_last_tick
g_chassis_cmd_rx_count
```

这些变量由 `defaultTask` 的回调写入，由 `controlTask` 周期读取。因为它们是 32-bit float/uint32_t，在 Cortex-M7 上单次读写通常是原子的，但多变量之间不是事务一致的。如果后续要做更严格的安全控制，建议把速度命令封装成结构体并加临界区或队列。

## 8. 运行优先级关系

按当前优先级，可以把运行关系理解为：

```text
中断
  > controlTask / canTask
  > defaultTask / imuTask / sbusTask / encoderTask
  > batteryTask / gpsTask / lidarTask
  > idleTask
```

具体含义：

| 情况 | 调度结果 |
| --- | --- |
| `controlTask` 10 ms 到期 | 它会优先运行，完成底盘控制后 `osDelay(10)` |
| CAN 帧到达 | 中断先入队，然后 `canTask` 以 AboveNormal 优先级运行 |
| `defaultTask` 正在 spin micro-ROS | 如果 `controlTask` 到期或 CAN 帧到达，会被抢占 |
| `imuTask` 读取最多等待 5 ms | 与 `defaultTask/sbusTask` 同优先级，可能影响 Normal 组内的可运行时间 |
| Low 优先级占位任务 | 只有更高优先级任务阻塞或 delay 时才运行 |

## 9. 一句话版时间安排

当前 FreeRTOS 的主节拍可以这样记：

| 周期 | 内容 |
| --- | --- |
| 1 ms | RTOS tick 和 HAL TIM6 时间基准 |
| 10 ms | 底盘控制、IMU 读取、SBUS 检查 |
| 事件触发 | CAN 反馈入队后立刻解析 |
| 200 ms | micro-ROS 发布 `/stm32_ping` 和 `/odom` |
| 500 ms | `/cmd_vel` 超时保护窗口 |
| 1000 ms | encoder/battery/gps/lidar 占位任务 delay |

如果后续要继续完善，最建议把 `batteryTask` 接入 ADC DMA 结果，把 `encoderTask` 接入 TIM2/TIM5 编码器计数，并在 `controlTask` 里统一做 `/cmd_vel`、SBUS、急停、failsafe 的控制源仲裁。
