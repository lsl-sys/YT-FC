/**
 * @file       propulsion.h
 * @author	   lsl-sys
 * @brief      Motor Driver (SUNNYSKY X2212 K1250)
 * @version    V3.0.0
 * @date       2025-11-23 2026-02-02
 * @Encoding   UTF-8
 */

#ifndef __PROPULSION_H
#define __PROPULSION_H

#include "main.h"

/* PWM配置: 50Hz标准电调频率，比较值范围2500-5000对应1000-2000us脉宽 */

#define PWM_MAX_COMPARE     5000    // 对应2000us (最大油门)
#define PWM_MIN_COMPARE     2500    // 对应1000us (最小油门/解锁)    

/* 电机输出限幅 (百分比 0.0-100.0) */
#define MOTOR_MAX_OUTPUT    70.0f   // 最大输出限制（保护电池/电机）
#define MOTOR_MIN_OUTPUT    0.0f    // 最小输出（停转）

/* 电机编号定义 (X型四旋翼布局) */
typedef enum {
    MOTOR_FL = 0,   // 前左 (Front-Left, M1)
    MOTOR_FR,       // 前右 (Front-Right, M2)
    MOTOR_BR,       // 后右 (Back-Right, M3)
    MOTOR_BL,       // 后左 (Back-Left, M4)
    MOTOR_COUNT     // 电机总数
} MotorID_t;

/* 电机PWM硬件句柄 */
typedef struct {
    TIM_HandleTypeDef *htim;    // 定时器句柄
    uint32_t channel;           // PWM通道 (TIM_CHANNEL_x)
} MotorHandle_t;

/* 电机转速结构 (百分比 0.0-100.0) */
typedef struct {
    float m1;   // FL
    float m2;   // FR
    float m3;   // BR
    float m4;   // BL
} MotorSpeed_t;

/** 初始化电机系统（配置PWM并发送解锁信号） */
void Propulsion_Init(const MotorHandle_t motors[MOTOR_COUNT]);

/** 混控输出：油门+姿态力矩→四电机转速（自动限幅与均衡） */
void Propulsion_MixOutput(float throttle, float pitch, float roll, float yaw);

/** 单电机强制设置（用于调试或单电机测试，绕开混控） */
void Propulsion_SetSingle(MotorID_t id, float speed);

/** 紧急停止（所有电机置最小油门，立即执行） */
void Propulsion_Stop(void);

/** 系统就绪检查：返回1表示已初始化且未故障，0表示未就绪（未解锁或故障） */
uint8_t Propulsion_IsReady(void);

#endif
