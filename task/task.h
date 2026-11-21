#ifndef TASK_H
#define TASK_H

#include <stdint.h>
#include "PID.h"
#include "RobStride2.h"

typedef struct {
    uint16_t MchanicalAngle;
    int16_t Speed;
    uint16_t LsatAngle;
    int16_t r;
    int32_t Angle;
    float Angle_DEG;
} M2006_TypeDef;

typedef struct {
    float torque;
    float angle;
    float omega;
    float kp;
    float kd;
} JointTarget_t;

/// 单个关节的机械标定：把电机坐标系换算到关节坐标系
///   关节角   q   = motor_inv * (θ_motor - pos_offset)
///   关节角速度 ω = motor_inv * ω_motor
///   关节力矩   τ = motor_inv * τ_motor
///   下发目标时反向换算：θ_motor = motor_inv * q + pos_offset，ω / τ 同乘 motor_inv
typedef struct {
    float pos_offset;  // 零点偏移 [rad]：关节处于机械零位时读到的电机角度
    int8_t motor_inv;  // 旋转方向：+1 与关节正方向一致，-1 相反
} JointCalib_t;

/// 轮子反馈（M2006 编码器）：固件 -> 上位机，8 B
typedef struct {
    float angle;   // 轮子角度 [rad]
    float omega;   // 轮子角速度 [rad/s]
} Wheel_t;

/// 轮子目标（上位机 -> 下位机）：与上位机 usb_drive/data_pack.h 的 WheelTarget_t 逐字节一致
///   torque : 轮轴力矩指令 [N*m]（唯一执行量：× WHEEL_COUNTS_PER_NM → 电流指令）
///   angle  : 保留字段（协议兼容，固件不再使用）
///   omega  : 保留字段（协议兼容，固件不再使用）
typedef struct {
    float angle;
    float omega;
    float torque;
} WheelTarget_t;

/// 下位机 -> 上位机：2×Wheel_t + 4×RobStrideState_t = 208 B
typedef struct {
    Wheel_t wheel[2];
    RobStrideState_t joint[4];
} RobotState_t;

/// 上位机 -> 下位机：2×WheelTarget_t + 4×JointTarget_t = 24 + 80 = 104 B
typedef struct {
    WheelTarget_t wheel[2];
    JointTarget_t joint[4];
} RobotTarget_t;

extern M2006_TypeDef wheel_motor[2];   // 左右轮 M2006（CAN 0x201 / 0x202）
extern RobotState_t rob_state;
extern RobotTarget_t rob_target;
extern RobStride_t joint_device[4];
extern JointCalib_t joint_calib[4];   // 四个关节的零点偏移/方向，按实机标定填写

extern volatile int wheel_cnt_send;
extern volatile int joint_cnt_send;
extern volatile uint32_t cdc_state_drop_cnt;   // 状态帧发送失败（USB 忙/未配置）计数

void M2006_Receive(M2006_TypeDef *dst, uint8_t *data);
int rm_send_2006(uint8_t *send_current, uint32_t id);
void motor_init_task(void *argument);   // 关节电机初始化（SetMode/Enable）
void wheel_control_task(void *argument);
void joint_control_task(void *argument);
void state_send_task(void *argument);
void target_receive_task(void *argument);

#endif
