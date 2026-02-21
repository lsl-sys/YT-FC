#include "pid_control.h"

FlightPIDSystem_t g_pid;

static inline float Abs(float x) { return x < 0 ? -x : x; }

static inline float Constrain(float val, float min, float max) {
    return (val < min) ? min : ((val > max) ? max : val);
}

/* 系统初始化：配置串级PID参数与保护阈值 */
void PID_InitAll(float hover_thr) {
    pidParam_t param;
    
    // 俯仰串级
    param = (pidParam_t){DEFAULT_PITCH_ANGLE_KP, DEFAULT_PITCH_ANGLE_KI, DEFAULT_PITCH_ANGLE_KD, DEFAULT_PITCH_ANGLE_ISEP};
    PID_Init(&g_pid.attitude.angle.pitch, 0.0f, param);
    param = (pidParam_t){DEFAULT_PITCH_RATE_KP, DEFAULT_PITCH_RATE_KI, DEFAULT_PITCH_RATE_KD, DEFAULT_PITCH_RATE_ISEP};
    PID_Init(&g_pid.attitude.rate.pitch, 0.0f, param);
		PID_SetOutputLimit(&g_pid.attitude.rate.pitch,20);
    
    // 横滚串级
    param = (pidParam_t){DEFAULT_ROLL_ANGLE_KP, DEFAULT_ROLL_ANGLE_KI, DEFAULT_ROLL_ANGLE_KD, DEFAULT_ROLL_ANGLE_ISEP};
    PID_Init(&g_pid.attitude.angle.roll, 0.0f, param);
    param = (pidParam_t){DEFAULT_ROLL_RATE_KP, DEFAULT_ROLL_RATE_KI, DEFAULT_ROLL_RATE_KD, DEFAULT_ROLL_RATE_ISEP};
    PID_Init(&g_pid.attitude.rate.roll, 0.0f, param);
		PID_SetOutputLimit(&g_pid.attitude.rate.roll,20);
    
    // 偏航（速率主控，角度环预留航向锁定）
    param = (pidParam_t){DEFAULT_YAW_RATE_KP, DEFAULT_YAW_RATE_KI, DEFAULT_YAW_RATE_KD, DEFAULT_YAW_RATE_ISEP};
    PID_Init(&g_pid.attitude.rate.yaw, 0.0f, param);
    param = (pidParam_t){DEFAULT_YAW_ANGLE_KP, DEFAULT_YAW_ANGLE_KI, DEFAULT_YAW_ANGLE_KD, 0.0f};
    PID_Init(&g_pid.attitude.angle.yaw, 0.0f, param);
    
    // 高度环
    param = (pidParam_t){DEFAULT_ALT_KP, DEFAULT_ALT_KI, DEFAULT_ALT_KD, DEFAULT_ALT_ISEP};
    PID_Init(&g_pid.altitude.alt, 0.0f, param);
    g_pid.altitude.hover_throttle = Constrain(hover_thr, 0.0f, 100.0f);
    
    g_pid.velocity.enabled = 0;
    g_pid.position.enabled = 0;
    
    g_pid.attitude.mode = MODE_ANGLE;
    g_pid.arm_flag = 0;
    g_pid.fault = 0;
    g_pid.out.pitch = g_pid.out.roll = g_pid.out.yaw = 0;
}

/* 系统复位：清空积分与输出（用于急停或模式切换） */
void PID_SystemReset(void) {
    PID_Reset(&g_pid.attitude.angle.pitch);
    PID_Reset(&g_pid.attitude.angle.roll);
    PID_Reset(&g_pid.attitude.angle.yaw);
    PID_Reset(&g_pid.attitude.rate.pitch);
    PID_Reset(&g_pid.attitude.rate.roll);
    PID_Reset(&g_pid.attitude.rate.yaw);
    PID_Reset(&g_pid.altitude.alt);
    
    g_pid.out.pitch = 0;
    g_pid.out.roll = 0;
    g_pid.out.yaw = 0;
    g_pid.fault = 0;
}

/* 切换控制模式（角度自稳/手动速率） */
void PID_SetMode(FlightMode_t mode) {
    g_pid.attitude.mode = mode;
}

/* 姿态控制主循环：串级PID计算，返回故障标志（1=倾角超限保护） */
uint8_t PID_UpdateAttitude(float target_pitch, float target_roll, float target_yaw,
                          float meas_pitch, float meas_roll, float meas_yaw,
                          float gyro_x, float gyro_y, float gyro_z) {
    float target_rate_pitch, target_rate_roll, target_rate_yaw;
    
    (void)meas_yaw; // 航向锁定功能预留
    
    /* 倾角保护：超限立即置故障标志，调用方需执行电机停转 */
    if (Abs(meas_pitch) > TILT_LIMIT_DEG || Abs(meas_roll) > TILT_LIMIT_DEG) {
        g_pid.fault = 1;
        return 1;
    }
    
    if (g_pid.attitude.mode == MODE_ANGLE)  {
        /* 外环：角度误差→角速度目标（注意：测量值与陀螺仪极性必须一致，否则正反馈炸机） */
        target_rate_pitch = PID_Calculate(&g_pid.attitude.angle.pitch, meas_pitch, target_pitch);
        target_rate_roll  = PID_Calculate(&g_pid.attitude.angle.roll,  meas_roll,  target_roll);
        
        target_rate_pitch = Constrain(target_rate_pitch, -MAX_RATE_TARGET_DPS, MAX_RATE_TARGET_DPS);
        target_rate_roll  = Constrain(target_rate_roll,  -MAX_RATE_TARGET_DPS, MAX_RATE_TARGET_DPS);
        
        target_rate_yaw = target_yaw; 
    } else {
        /* 手动模式：摇杆直接映射为角速度 */
        target_rate_pitch = target_pitch;
        target_rate_roll  = target_roll;
        target_rate_yaw   = target_yaw;
    }
    
    /* 内环：角速度误差→力矩输出（gyro_x对应pitch，极性错误会导致抬头加速抬头） */
    g_pid.out.pitch = PID_Calculate(&g_pid.attitude.rate.pitch, gyro_x, target_rate_pitch);
    g_pid.out.roll  = PID_Calculate(&g_pid.attitude.rate.roll,  gyro_y, target_rate_roll);
    g_pid.out.yaw   = PID_Calculate(&g_pid.attitude.rate.yaw,   gyro_z, target_rate_yaw);
    
    return 0;
}

/* 获取姿态PID输出（单位：力矩/油门混合量，用于电机混控） */
void PID_GetAttitudeOutput(float *pitch_out, float *roll_out, float *yaw_out) {
    if (pitch_out) *pitch_out = g_pid.out.pitch;
    if (roll_out)  *roll_out  = g_pid.out.roll;
    if (yaw_out)   *yaw_out   = g_pid.out.yaw;
}

/* 高度控制：返回油门补偿量（需叠加悬停油门） */
float PID_UpdateAlt(float target_alt, float meas_alt) {
    float output = PID_Calculate(&g_pid.altitude.alt, meas_alt, target_alt);
    return Constrain(output, -MAX_ALT_OUTPUT, MAX_ALT_OUTPUT);
}

/* 在线调整角度环参数（调参/自适应用） */
void PID_SetAngleParam(PID_Axis_t axis, const pidParam_t* param) {
    if (!param) return;
    PIDController *pc = NULL;
    switch (axis) {
        case AXIS_PITCH: pc = &g_pid.attitude.angle.pitch; break;
        case AXIS_ROLL:  pc = &g_pid.attitude.angle.roll;  break;
        case AXIS_YAW:   pc = &g_pid.attitude.angle.yaw;   break;
        default: return;
    }
    pc->kp = param->kp; 
    pc->ki = param->ki; 
    pc->kd = param->kd; 
    pc->iSepThresh = param->iSepThresh;
}

/* 在线调整角速度环参数 */
void PID_SetRateParam(PID_Axis_t axis, const pidParam_t* param) {
    if (!param) return;
    PIDController *pc = NULL;
    switch (axis) {
        case AXIS_PITCH: pc = &g_pid.attitude.rate.pitch; break;
        case AXIS_ROLL:  pc = &g_pid.attitude.rate.roll;  break;
        case AXIS_YAW:   pc = &g_pid.attitude.rate.yaw;   break;
        default: return;
    }
    pc->kp = param->kp; 
    pc->ki = param->ki; 
    pc->kd = param->kd; 
    pc->iSepThresh = param->iSepThresh;
}

/* 在线调整高度环参数 */
void PID_SetAltParam(const pidParam_t* param) {
    if (!param) return;
    g_pid.altitude.alt.kp = param->kp;
    g_pid.altitude.alt.ki = param->ki;
    g_pid.altitude.alt.kd = param->kd;
    g_pid.altitude.alt.iSepThresh = param->iSepThresh;
}

/* 倾角安全检测：超过TILT_LIMIT_DEG返回1，用于触发迫降保护 */
uint8_t PID_CheckTilt(float pitch, float roll) {
    return (Abs(pitch) > TILT_LIMIT_DEG || Abs(roll) > TILT_LIMIT_DEG) ? 1 : 0;
}

/* 摇杆量→目标角度映射（-100~100 → -MAX_ANGLE_TARGET~MAX_ANGLE_TARGET） */
float PID_StickToAngle(int16_t stick) {
    return Constrain((float)stick, -100.0f, 100.0f) * (MAX_ANGLE_TARGET / 100.0f);
}

/* 摇杆量→目标角速度映射（-100~100 → -MAX_RATE_TARGET_DPS~MAX_RATE_TARGET_DPS） */
float PID_StickToRate(int16_t stick) {
    return Constrain((float)stick, -100.0f, 100.0f) * (MAX_RATE_TARGET_DPS / 100.0f);
}
