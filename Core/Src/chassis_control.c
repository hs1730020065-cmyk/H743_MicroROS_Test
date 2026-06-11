/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    chassis_control.c
  * @brief   自定义控制板的阿克曼底盘指令换算与下发。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "chassis_control.h"
#include "can_motor.h"
#include "can_steer.h"
#include <math.h>

/* 车辆机械参数：轴距 L = 0.660 m。 */
#define CHASSIS_WHEEL_BASE_M          0.660f

/* 车辆机械参数：轮距 W = 0.665 m，当前单前轮转角模型暂时不用，后续可用于左右轮角度修正。 */
#define CHASSIS_TRACK_WIDTH_M         0.665f

/* 轮胎直径 D = 0.420 m，当前直接输出 m/s，后续换算电机转速时使用。 */
#define CHASSIS_TIRE_DIAMETER_M       0.420f

/* 后驱电机目标速度限幅：最高速度约 2.22 m/s。 */
#define CHASSIS_MAX_SPEED_MPS         2.22f

/* 最大车轮转角 30 deg（实测 EPS 最大支持 30°）。 */
#define CHASSIS_MAX_STEER_ANGLE_DEG   30.0f

/* linear_x 过小时不再计算 atan(L*w/v)，避免除以接近 0 的速度。 */
#define CHASSIS_LINEAR_X_EPS_MPS      0.01f

/* 弧度转角度系数：180 / pi。 */
#define CHASSIS_RAD_TO_DEG            57.2957795f

/*
 * 转向电机协议的正方向与 ROS2 angular.z 的正方向相反。
 * ROS2 中 angular.z > 0 表示左转；当前 EPS 正角度实际表现为右转，
 * 因此这里统一乘 -1，让键盘/ROS2 的左转右转和实车方向一致。
 */
#define CHASSIS_STEER_DIRECTION_SIGN  (-1.0f)

/* raw_abs = round(abs(speed_mps) * 454.5). Direction is encoded in can_motor.c. */
#define CHASSIS_DRIVE_SPEED_TO_RAW_SCALE  454.5f
#define CHASSIS_DRIVE_MAX_RAW_ABS         0x8000U

/* 后驱电机当前目标车速，供调试观察或 micro-ROS 状态反馈使用。 */
volatile float g_drive_motor_target_speed_mps = 0.0f;
volatile float g_drive_motor_actual_speed_mps = 0.0f;
volatile uint16_t g_drive_motor_actual_speed_raw = 0U;
volatile uint32_t g_drive_motor_feedback_rx_count = 0U;
volatile uint32_t g_drive_motor_feedback_last_tick = 0U;
/* 转向电机当前目标角度，供调试观察或 micro-ROS 状态反馈使用。 */
volatile float g_steer_motor_target_angle_deg = 0.0f;
/* 转向电机当前实际角度（来自 EPS 0x18F 反馈），单位：度。 */
volatile float g_steer_motor_actual_angle_deg = 0.0f;
/* 车辆当前实际横摆角速度（由阿克曼模型解算），单位：弧度/秒。 */
volatile float g_chassis_actual_angular_z = 0.0f;
/* 最近一次后驱电机 CAN 下发状态。 */
volatile HAL_StatusTypeDef g_drive_motor_can_status = HAL_OK;
/* 最近一次转向电机 CAN 下发状态。 */
volatile HAL_StatusTypeDef g_steer_motor_can_status = HAL_OK;

/**
 * @brief       计算浮点数绝对值
 * @param       value: 需要取绝对值的浮点数
 * @retval      value 的绝对值
 */
static float Chassis_AbsFloat(float value)
{
  return (value >= 0.0f) ? value : -value;
}

/**
 * @brief       将浮点数限制在指定范围内
 * @param       value    : 输入值
 *              min_value: 最小允许值
 *              max_value: 最大允许值
 * @retval      限幅后的浮点数
 */
static float Chassis_LimitFloat(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }

  if (value < min_value)
  {
    return min_value;
  }

  return value;
}

/**
 * Convert ROS2 /cmd_vel linear speed to motor raw magnitude.
 * raw_abs = round(abs(speed_mps) * 454.5).
 * Direction is encoded later by CanMotor_Forward()/CanMotor_Reverse().
 */
static uint16_t Chassis_SpeedMpsToRaw(float speed_mps)
{
  float speed_abs;
  float raw_float;

  speed_abs = Chassis_AbsFloat(speed_mps);
  speed_abs = Chassis_LimitFloat(speed_abs, 0.0f, CHASSIS_MAX_SPEED_MPS);

  raw_float = speed_abs * CHASSIS_DRIVE_SPEED_TO_RAW_SCALE;

  if (raw_float <= 0.0f)
  {
    return 0U;
  }

  if (raw_float >= (float)CHASSIS_DRIVE_MAX_RAW_ABS)
  {
    return CHASSIS_DRIVE_MAX_RAW_ABS;
  }

  return (uint16_t)(raw_float + 0.5f);
}

float DriveMotor_RawFeedbackToSpeed(uint16_t raw_speed)
{
  uint16_t speed_abs_raw;
  float speed_mps;

  if (raw_speed == 0U)
  {
    return 0.0f;
  }

  /*
   * 补码协议:
   * raw >= 0x8000: 正转 (two's complement 负数)
   *   0x8000 = -32768 = 最快正转
   *   0xFFFF = -1     = 最慢正转
   * raw <  0x8000: 反转 (two's complement 非负数)
   *   0x0001 = +1     = 最慢反转
   *   0x7FFF = +32767 = 最快反转
   */
  if (raw_speed >= 0x8000U)
  {
    speed_abs_raw = (uint16_t)(0x10000UL - (uint32_t)raw_speed);
    speed_mps = (float)speed_abs_raw / CHASSIS_DRIVE_SPEED_TO_RAW_SCALE;
  }
  else
  {
    speed_abs_raw = raw_speed;
    speed_mps = -((float)speed_abs_raw / CHASSIS_DRIVE_SPEED_TO_RAW_SCALE);
  }

  return speed_mps;
}

/**
 * @brief       将浮点转向角四舍五入成 CAN 协议使用的整数角度
 * @param       angle_deg: 前轮目标转向角，单位：度（deg）
 * @retval      四舍五入后的 int16_t 角度
 * @note        can_steer.c 当前接口是 EpsSteer_SendTargetAngle(int16_t angle_deg)，
 *              因此这里先按“1 raw = 1 度”下发。
 */
static int16_t Chassis_AngleDegToInt16(float angle_deg)
{
  if (angle_deg >= 0.0f)
  {
    return (int16_t)(angle_deg + 0.5f);
  }

  return (int16_t)(angle_deg - 0.5f);
}

/**
 * @brief       根据 ROS2/cmd_vel 指令计算阿克曼底盘目标量
 * @param       linear_x : 车辆纵向线速度，单位：米/秒（m/s）
 *              angular_z: 车辆绕 Z 轴角速度，单位：弧度/秒（rad/s）
 *              out      : 输出结果，包含后驱目标速度和前轮目标转向角
 * @retval      无
 * @note        阿克曼转角公式为 delta = atan(L * angular_z / linear_x)。
 *              当 linear_x 接近 0 时按停车处理，避免除以接近 0 的速度。
 */
void Ackermann_Calc(float linear_x, float angular_z, AckermannTarget_t *out)
{
  float speed_mps;
  float steer_rad;
  float steer_deg;

  if (out == NULL)
  {
    return;
  }

  /*
   * 先限制后驱目标速度，保证给到底层执行器的速度不会超过车辆能力。
   * 这里保留正负号：正值前进，负值后退。
   */
  speed_mps = Chassis_LimitFloat(linear_x,
                                -CHASSIS_MAX_SPEED_MPS,
                                 CHASSIS_MAX_SPEED_MPS);

  /*
   * 速度接近 0 时，角速度无法转换成稳定的阿克曼转角。
   * 这种情况下车辆按停止处理，转向角归零，避免除以接近 0 的速度。
   */
  if (Chassis_AbsFloat(speed_mps) < CHASSIS_LINEAR_X_EPS_MPS)
  {
    out->rear_target_speed_mps = 0.0f;
    out->steer_target_angle_deg = 0.0f;
    return;
  }

  /*
   * 阿克曼单轨模型：
   * delta = atan(L * angular_z / linear_x)
   * 这里使用限幅后的 speed_mps 参与计算，使转向角和最终下发速度一致。
   */
  steer_rad = atanf((CHASSIS_WHEEL_BASE_M * angular_z) / speed_mps);
  steer_deg = steer_rad * CHASSIS_RAD_TO_DEG * CHASSIS_STEER_DIRECTION_SIGN;

  /*
   * 转向角按最小转弯半径限制到 +/-21.2 度，
   * 防止上位机给出过大的 angular_z 时超过机械极限。
   */
  steer_deg = Chassis_LimitFloat(steer_deg,
                                -CHASSIS_MAX_STEER_ANGLE_DEG,
                                 CHASSIS_MAX_STEER_ANGLE_DEG);

  out->rear_target_speed_mps = speed_mps;
  out->steer_target_angle_deg = steer_deg;
}

/**
 * @brief       根据 ROS2/cmd_vel 指令执行一次底盘控制计算和下发
 * @param       linear_x : 车辆纵向线速度，单位：米/秒（m/s）
 *              angular_z: 车辆绕 Z 轴角速度，单位：弧度/秒（rad/s）
 * @retval      无
 * @note        适合放在 FreeRTOS 10ms 控制任务中周期调用。
 */
void Chassis_ControlFromCmdVel(float linear_x, float angular_z)
{
  AckermannTarget_t target;

  /*
   * 上位机/ROS2 的 cmd_vel 先转换成底盘执行目标，
   * 再统一交给后驱电机和转向电机接口。
   */
  Ackermann_Calc(linear_x, angular_z, &target);

  DriveMotor_SetSpeed(target.rear_target_speed_mps);
  SteerMotor_SetAngle(target.steer_target_angle_deg);
}

/**
 * @brief       设置后驱电机目标速度
 * @param       speed_mps: 后驱电机目标车速，单位：米/秒（m/s）
 * @retval      无
 * @note        当前只是保存目标值，方便 micro-ROS 反馈和调试观察。
 *              后续可在这里补充 CAN、PWM 或 UART 等实际下发协议。
 */
void DriveMotor_SetSpeed(float speed_mps)
{
  float limited_speed_mps;
  uint16_t raw_speed_abs;

  /*
   * 后驱电机底层协议预留接口。
   * 后续可在这里补 CAN/PWM/UART 下发逻辑；当前先保存目标值，方便调试观察。
   */
  limited_speed_mps = Chassis_LimitFloat(speed_mps,
                                        -CHASSIS_MAX_SPEED_MPS,
                                         CHASSIS_MAX_SPEED_MPS);
  g_drive_motor_target_speed_mps = limited_speed_mps;

  /*
   * 将 m/s 换成电机协议需要的 raw_speed 绝对值。
   * 方向仍由正负号决定：正数前进，负数后退，接近 0 时发停止帧。
   */
  raw_speed_abs = Chassis_SpeedMpsToRaw(limited_speed_mps);

  if (raw_speed_abs == 0U)
  {
    g_drive_motor_can_status = CanMotor_Stop();
  }
  else if (limited_speed_mps > 0.0f)
  {
    g_drive_motor_can_status = CanMotor_Forward(raw_speed_abs);
  }
  else
  {
    g_drive_motor_can_status = CanMotor_Reverse(raw_speed_abs);
  }
}

/**
 * @brief       设置前轮转向电机目标角度
 * @param       angle_deg: 前轮目标转向角，单位：度（deg）
 * @retval      无
 * @note        当前只是保存目标值，方便 micro-ROS 反馈和调试观察。
 *              后续可在这里补充转向电机闭环控制或通信协议。
 */
void SteerMotor_SetAngle(float angle_deg)
{
  float limited_angle_deg;
  int16_t angle_raw_deg;

  /*
   * 前轮转向电机底层协议预留接口。
   * 后续可在这里补转向电机闭环或通信协议；当前先保存目标角度。
   */
  limited_angle_deg = Chassis_LimitFloat(angle_deg,
                                        -CHASSIS_MAX_STEER_ANGLE_DEG,
                                         CHASSIS_MAX_STEER_ANGLE_DEG);
  g_steer_motor_target_angle_deg = limited_angle_deg;

  /*
   * 转向 CAN 接口目前以整数角度下发。
   * 如果后续 EPS 协议确认是 0.1 度或其他单位，只需要改这里的换算。
   */
  angle_raw_deg = Chassis_AngleDegToInt16(limited_angle_deg);
  g_steer_motor_can_status = EpsSteer_SendTargetAngle(angle_raw_deg);
}

/**
 * @brief       根据实际线速度和转向角计算车辆实际横摆角速度（阿克曼单轨模型）
 * @param       linear_x_mps : 车辆实际纵向线速度，单位：米/秒（m/s）
 *              steer_angle_deg: 实际前轮转向角，单位：度（deg）
 *              正值 = 右转（EPS 协议方向），负值 = 左转
 * @retval      车辆绕 Z 轴的实际角速度，单位：弧度/秒（rad/s）
 *              正值 = 左转（ROS 坐标系），负值 = 右转
 * @note        ω = v * tan(δ) / L
 *              EPS 反馈角度为正时右转，ROS angular.z 为正时左转，
 *              因此输出需要取反。
 *              当线速度接近 0 时，车辆无法产生有意义的横摆角速度，返回 0。
 */
float Chassis_CalcActualYawRate(float linear_x_mps, float steer_angle_deg)
{
  float speed_abs;
  float steer_rad;
  float yaw_rate;

  speed_abs = Chassis_AbsFloat(linear_x_mps);

  /* 车速极低时没有有意义的横摆角速度。 */
  if (speed_abs < CHASSIS_LINEAR_X_EPS_MPS)
  {
    return 0.0f;
  }

  /* 角度 -> 弧度，方向取反以匹配 ROS 坐标系。 */
  steer_rad = steer_angle_deg * (1.0f / CHASSIS_RAD_TO_DEG) * CHASSIS_STEER_DIRECTION_SIGN;

  /* 阿克曼单轨模型: ω = v * tan(δ) / L。 */
  yaw_rate = linear_x_mps * tanf(steer_rad) / CHASSIS_WHEEL_BASE_M;

  return yaw_rate;
}
