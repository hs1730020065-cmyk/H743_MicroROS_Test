/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    chassis_control.h
  * @brief   Ackermann chassis command conversion interface.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  float rear_target_speed_mps;
  float steer_target_angle_deg;
} AckermannTarget_t;

extern volatile float g_drive_motor_target_speed_mps;
extern volatile float g_steer_motor_target_angle_deg;
extern volatile HAL_StatusTypeDef g_drive_motor_can_status;
extern volatile HAL_StatusTypeDef g_steer_motor_can_status;

/**
 * @brief       根据 ROS2/cmd_vel 指令计算阿克曼底盘目标量
 * @param       linear_x : 车辆纵向线速度，单位：米/秒（m/s）
 *              angular_z: 车辆绕 Z 轴角速度，单位：弧度/秒（rad/s）
 *              out      : 输出结果，包含后驱目标速度和前轮目标转向角
 * @retval      无
 */
void Ackermann_Calc(float linear_x, float angular_z, AckermannTarget_t *out);

/**
 * @brief       根据 ROS2/cmd_vel 指令执行一次底盘控制计算和下发
 * @param       linear_x : 车辆纵向线速度，单位：米/秒（m/s）
 *              angular_z: 车辆绕 Z 轴角速度，单位：弧度/秒（rad/s）
 * @retval      无
 */
void Chassis_ControlFromCmdVel(float linear_x, float angular_z);

/**
 * @brief       设置后驱电机目标速度
 * @param       speed_mps: 后驱电机目标车速，单位：米/秒（m/s）
 * @retval      无
 */
void DriveMotor_SetSpeed(float speed_mps);

/**
 * @brief       设置前轮转向电机目标角度
 * @param       angle_deg: 前轮目标转向角，单位：度（deg）
 * @retval      无
 */
void SteerMotor_SetAngle(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_CONTROL_H */
