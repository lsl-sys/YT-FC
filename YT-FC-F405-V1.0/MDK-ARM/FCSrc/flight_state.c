#include "flight_state.h"
#include "pid_control.h"    
#include "RemoteControl.h"  
#include "Buzzer.h"  
#include "ELRS.h"          
#include "imu.h"         

extern imu_data_t imu; 
extern Buzzer_HandleTypeDef buzzer;
extern bool buzzer_armed_done;

FlightState_t g_fstate = {
    .state = STATE_DISARMED,
    .last_state = STATE_DISARMED,
    .state_enter_tick = 0,
    .is_first_arm = 1,
    .last_rc_tick = 0,
    .last_imu_tick = 0
};

/* 摇杆阈值（美国手） */
#define STICK_HIGH      80
#define STICK_LOW      -80

/**
 * @brief  检查内八姿势（左摇杆：右+下）
 * @note   简化版内八，只用左摇杆（右手可保持握持）
 */
static uint8_t check_inner_arm(void)
{
    return (filtered_rc.LX > STICK_HIGH) && (filtered_rc.LY < STICK_LOW);
}

/**
 * @brief  检查外八姿势（左摇杆：左+下）
 */
static uint8_t check_outer_disarm(void)
{
    return (filtered_rc.LX < STICK_LOW) && (filtered_rc.LY < STICK_LOW);
}

/**
 * @brief  检查遥控器是否在线（500ms超时）
 */
static uint8_t check_rc_online(void)
{
    uint32_t now = HAL_GetTick();
    if (elrs_is_connected() && (now - g_fstate.last_rc_tick < 500)) {
        return 1;
    }
    return 0;
}

/**
 * @brief  状态进入时的初始化动作
 */
static void on_state_enter(ArmState_t new_state)
{
    g_fstate.state_enter_tick = HAL_GetTick();
    g_fstate.last_state = g_fstate.state;
    g_fstate.state = new_state;
    
    switch (new_state) {
        case STATE_DISARMED:
            g_pid.arm_flag = 0;
            // 可选：短提示音
            break;
            
        case STATE_PRE_ARM:
            // 开始计时，无其他动作（等待用户保持2秒后松手）
            break;
            
        case STATE_ARMED:
            g_pid.arm_flag = 1;
            g_fstate.is_first_arm = 0;
            PID_SystemReset();  // 解锁时清零PID，防止积分突变
            break;
            
        case STATE_EMERGENCY:
            g_pid.arm_flag = 0;
            g_pid.out.throttle = 0;
            g_pid.out.pitch = 0;
            g_pid.out.roll = 0;
            g_pid.out.yaw = 0;
            PID_SystemReset();
            break;
    }
}

void FState_Update(void)
{
    uint32_t now = HAL_GetTick();
    
    /* 更新时间戳 */
    if (elrs_is_connected()) {
        g_fstate.last_rc_tick = now;
    }
    if (imu.online) {
        g_fstate.last_imu_tick = now;
    }
    
    switch (g_fstate.state) {
        
        /* ==================== DISARMED：锁定状态 ==================== */
        case STATE_DISARMED:
        {
            if (!FState_CanArm() || !check_rc_online()) {
                return; // 不满足条件，保持锁定
            }
            
            // 检测到内八，进入准备状态
            if (check_inner_arm()) {
                on_state_enter(STATE_PRE_ARM);
            }
            break;
        }
        
        /* ==================== PRE_ARM：准备解锁（关键修改）==================== */
        case STATE_PRE_ARM:
        {
            uint32_t hold_time = now - g_fstate.state_enter_tick;
            uint8_t is_holding = check_inner_arm();
            
            // 检查条件：RC必须在线
            if (!check_rc_online()) {
                on_state_enter(STATE_DISARMED);
                return;
            }
            
            // 场景1：已满2秒且用户松手 → 解锁成功
            if (hold_time >= 2000 && !is_holding) {
                on_state_enter(STATE_ARMED);
                return;
            }
            
            // 场景2：未满2秒但用户松手 → 解锁失败，回到锁定
            if (hold_time < 2000 && !is_holding) {
                on_state_enter(STATE_DISARMED);
                return;
            }
            
            // 场景3：保持内八满2秒 → 继续等待松手（可添加提示音表示"已就绪"）
            if (hold_time >= 2000 && is_holding) {
                // 可选：Buzzer_Beep(&buzzer, 50); // 短促提示：已准备好，请松手
            }
            
            // 场景4：保持内八未满2秒 → 继续等待
            break;
        }
        
        /* ==================== ARMED：已解锁（飞行中）==================== */
        case STATE_ARMED:
        {
            // 安全检查
            if (!check_rc_online() || !imu.online || !imu.valid) {
                on_state_enter(STATE_EMERGENCY);
                return;
            }
            
            // 外八上锁
            if (check_outer_disarm()) {
                on_state_enter(STATE_EMERGENCY);
                return;
            }
            
            // 倾角过大保护（由PID_CheckTilt触发）
            break;
        }
        
        /* ==================== EMERGENCY：紧急锁定 ==================== */
        case STATE_EMERGENCY:
        {
            // 必须松开外八才能回到DISARMED
            if (!check_outer_disarm()) {
                if (now - g_fstate.state_enter_tick > 500) {
                    on_state_enter(STATE_DISARMED);
                }
            }
            break;
        }
    }
}

ArmState_t FState_GetState(void)
{
    return g_fstate.state;
}

void FState_ForceEmergency(void)
{
    if (g_fstate.state != STATE_EMERGENCY) {
        on_state_enter(STATE_EMERGENCY);
    }
}

uint8_t FState_CanArm(void)
{
    if (!imu.online || !imu.valid) return 0;
    return !PID_CheckTilt(imu.pitch, imu.roll);
}
