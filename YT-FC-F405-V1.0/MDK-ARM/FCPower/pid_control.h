/**
 * @file       pid_control.h
 * @author     lsl-sys
 * @brief      Multi-axis PID Control System (Altitude/Velocity/Attitude Loops)
 * @version    V2.2.0
 * @date       2026-02-01 2026-02-07
 */

#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

#include "main.h"
#include "pid_core.h"

/* ========== 默认参数 ========== */

// 俯仰/横滚角度环（自稳模式用）
#define DEFAULT_PITCH_ANGLE_KP      0.4f
#define DEFAULT_PITCH_ANGLE_KI      0.0f
#define DEFAULT_PITCH_ANGLE_KD      0.00f
#define DEFAULT_PITCH_ANGLE_ISEP    20.0f

#define DEFAULT_ROLL_ANGLE_KP       0.4f
#define DEFAULT_ROLL_ANGLE_KI       0.0f
#define DEFAULT_ROLL_ANGLE_KD       0.00f
#define DEFAULT_ROLL_ANGLE_ISEP     20.0f

// 偏航角度环（不用）
#define DEFAULT_YAW_ANGLE_KP        1.0f
#define DEFAULT_YAW_ANGLE_KI        0.0f
#define DEFAULT_YAW_ANGLE_KD        0.0f
#define DEFAULT_YAW_ANGLE_ISEP      0.0f

// 角速度环（内环）
#define DEFAULT_PITCH_RATE_KP       0.75f
#define DEFAULT_PITCH_RATE_KI       0.0f
#define DEFAULT_PITCH_RATE_KD       0.0f
#define DEFAULT_PITCH_RATE_ISEP     0.0f

#define DEFAULT_ROLL_RATE_KP        0.75f
#define DEFAULT_ROLL_RATE_KI        0.0f
#define DEFAULT_ROLL_RATE_KD        0.0f
#define DEFAULT_ROLL_RATE_ISEP      0.0f

#define DEFAULT_YAW_RATE_KP         0.0f
#define DEFAULT_YAW_RATE_KI         0.0f
#define DEFAULT_YAW_RATE_KD         0.0f
#define DEFAULT_YAW_RATE_ISEP       0.0f

// 高度环
#define DEFAULT_ALT_KP              1.0f
#define DEFAULT_ALT_KI              0.3f
#define DEFAULT_ALT_KD              0.0f
#define DEFAULT_ALT_ISEP            0.0f

// 保护阈值与限幅
#define TILT_LIMIT_DEG              45.0f //倾倒保护阈值
#define MAX_ANGLE_TARGET            30.0f //最大目标姿态角
#define MAX_ALT_OUTPUT              30.0f //高度环输出限幅
#define MAX_RATE_TARGET_DPS         200.0f//最大目标角速度

/* ========== 类型定义 ========== */

typedef enum {
    MODE_ANGLE = 0, //自稳模式
    MODE_RATE,      //手动模式
} FlightMode_t;

typedef enum {
    AXIS_PITCH = 0,
    AXIS_ROLL,
    AXIS_YAW,
    AXIS_COUNT
} PID_Axis_t;

typedef struct {
    PIDController pitch;
    PIDController roll;
    PIDController yaw;
} RatePID_t;

typedef struct {
    PIDController pitch;
    PIDController roll;
    PIDController yaw;
} AnglePID_t;

typedef struct {
    AnglePID_t angle;
    RatePID_t  rate;
    FlightMode_t mode;
} AttitudePID_t;

typedef struct {
    PIDController alt;
    float hover_throttle;
} AltitudePID_t;

typedef struct {
    PIDController x;
    PIDController y;
    uint8_t enabled;
} VelocityPID_t;

typedef struct {
    PIDController x;
    PIDController y;
    uint8_t enabled;
} PositionPID_t;

typedef struct {
    AttitudePID_t attitude;
    AltitudePID_t altitude;
    VelocityPID_t velocity;
    PositionPID_t position;
    
    struct {
        float pitch;
        float roll;
        float yaw;
			  float throttle;
    } out;            // 姿态控制输出
	
    uint8_t arm_flag; //解锁标志（0=锁定，1=已解锁，电机允许转动）
    uint8_t fault;    //故障标志（0=正常，1=触发倾角保护等故障）
} FlightPIDSystem_t;

// 初始化，hover_thr范围 0.0-100.0
void PID_InitAll(float hover_thr);

// 系统紧急复位（清空积分，清除故障）
void PID_SystemReset(void);

// 设置飞行模式
void PID_SetMode(FlightMode_t mode);

// 姿态控制（MODE_ANGLE: target为角度；MODE_RATE: target为角速度）
// 返回0正常，1表示触发倾角保护
uint8_t PID_UpdateAttitude(float target_pitch, float target_roll, float target_yaw,
                          float meas_pitch, float meas_roll, float meas_yaw,
                          float gyro_x, float gyro_y, float gyro_z);

// 获取姿态控制输出（三个轴的力矩，范围约-100~100）
void PID_GetAttitudeOutput(float *pitch_out, float *roll_out, float *yaw_out);
													
// 高度控制，返回油门修正量(需叠加到基础油门)
float PID_UpdateAlt(float target_alt, float meas_alt);

// 设置PID参数（用于解锁前检查和飞行中保护）
void PID_SetAngleParam(PID_Axis_t axis, const pidParam_t* param);
void PID_SetRateParam(PID_Axis_t axis, const pidParam_t* param);
void PID_SetAltParam(const pidParam_t* param);

// 检查倾角是否超限，返回1表示超限
uint8_t PID_CheckTilt(float pitch, float roll);

// 摇杆映射
float PID_StickToAngle(int16_t stick);   // -100~100 -> -30~30度
float PID_StickToRate(int16_t stick);    // -100~100 -> -300~300度/秒

extern FlightPIDSystem_t g_pid;

#endif
