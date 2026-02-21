/**
 * @file       pid_core.h
 * @author     lsl-sys
 * @brief      Core PID Algorithm (Pure Mathematical Implementation) Generic controller foundation without application-specific logic
 * @version    V2.0.0
 * @date       2025-08-01 2026-2-1
 * @Encoding   UTF-8
 */

#ifndef __PID_CORE_H
#define __PID_CORE_H

#include "main.h"

typedef struct {
    float kp;       
    float ki;       
    float kd;       
	  float iSepThresh;   // 积分分离阈值，0表示禁用积分分离
} pidParam_t;

/* 默认参数*/
#define DEFAULT_PID_INTEGRATION_LIMIT  20.0f
#define DEFAULT_PID_OUTPUT_LIMIT       100.0f 
#define DEFAULT_PID_DEAD_BAND          0.0f    
#define DEFAULT_PID_MAX_ERR            0.0f    // 最大误差限幅，0表示无限制


typedef struct {
    float desired;      // 目标值
    float measure;      // 测量值
	  float prevMeasure;  // 上次测量值，用于微分（Derivative on Measurement）

    float error;        // 当前误差：desired - measure

    float kp;           // 比例增益
    float ki;           // 积分增益
    float kd;           // 微分增益

    float outP;         // 比例输出
    float outI;         // 积分输出
    float outD;         // 微分输出
    float out;          // 总输出

    float integ;        // 积分累积值
    float deriv;        // 微分计算结果

    float iLimit;       // 积分限幅绝对值
    float outputLimit;  // 输出限幅绝对值

    float deadBand;     // 死区阈值：|error|<死区时输出0
    float maxErr;       // 最大误差限制
		float iSepThresh;   // 积分分离阈值（运行时也可单独设置）
    
    uint8_t firstUpdate;// 首次调用标志，用于消除初始微分冲击

} PIDController;




/* 生命周期管理 */
/** 初始化PID控制器，设置目标值与PID参数 */
void PID_Init(PIDController* pid, const float desired, const pidParam_t pidParam);

/** 计算PID输出（控制周期调用，内部处理微分先行与积分分离） */
float PID_Calculate(PIDController* pid, float measure, float target);

/** 重置积分、微分及首次调用标志（切换模式或异常恢复时使用） */
void PID_Reset(PIDController* pid);




/* 参数动态调整（支持在线调参，无需重新初始化） */
/** 设置积分限幅值（防止积分饱和，绝对值） */
void PID_SetIntegralLimit(PIDController* pid, const float limit);

/** 设置输出限幅值（绝对值） */
void PID_SetOutputLimit(PIDController* pid, const float limit);

/** 更新目标设定值（支持动态目标跟踪） */
void PID_SetDesired(PIDController* pid, const float desired);

/** 设置比例系数Kp */
void PID_SetKp(PIDController* pid, const float kp);

/** 设置积分系数Ki */
void PID_SetKi(PIDController* pid, const float ki);

/** 设置微分系数Kd */
void PID_SetKd(PIDController* pid, const float kd);

/** 设置控制死区阈值（|error|<阈值时输出保持，减少抖动） */
void PID_SetDeadBand(PIDController* pid, const float deadBand);

/** 设置最大误差限制（超限触发保护，输出清零） */
void PID_SetMaxErr(PIDController* pid, const float maxErr);

/** 设置积分分离阈值（|error|>阈值时暂停积分累积，改善大偏差响应） */
void PID_SetISepThresh(PIDController* pid, const float thresh);

#endif

