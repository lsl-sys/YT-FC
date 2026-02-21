/**
 * @file       flight_state.h
 * @author     lsl-sys
 * @brief      Flight State Manager 
 * @version    V2.0.0
 * @date       2026-02-01 2026-02-018
 */


#ifndef __FLIGHT_STATE_H
#define __FLIGHT_STATE_H

#include "main.h"

/* 解锁状态机 */
typedef enum {
    STATE_DISARMED = 0,     // 锁定：电机禁止，等待解锁姿势
    STATE_PRE_ARM,          // 准备解锁：检测到内八，开始2秒计时
    STATE_ARMED,            // 已解锁：允许飞行控制
    STATE_EMERGENCY         // 紧急锁定：RC丢失/倾角过大/外八上锁，需重新解锁
} ArmState_t;

/* 全局状态结构 */
typedef struct {
    ArmState_t state;           // 当前状态
    uint32_t state_enter_tick;  // 进入当前状态的时间戳（用于防抖）
    uint8_t is_first_arm;       // 首次解锁标志（用于提示音）
    ArmState_t last_state;      // 上次状态（用于检测状态变化）
    
    /* 安全监控 */
    uint32_t last_rc_tick;      // 上次收到有效RC的时间
    uint32_t last_imu_tick;     // 上次有效IMU的时间
} FlightState_t;

extern FlightState_t g_fstate;

/**
 * @brief  状态机更新（非阻塞，快速返回）
 * @note   需在主循环中高频调用（如200Hz）
 */
void FState_Update(void);

/**
 * @brief  获取当前状态
 */
ArmState_t FState_GetState(void);

/**
 * @brief  强制紧急状态（倾角过大时调用）
 */
void FState_ForceEmergency(void);

/**
 * @brief  检查是否满足解锁条件（水平+RC在线）
 */
uint8_t FState_CanArm(void);

#endif
