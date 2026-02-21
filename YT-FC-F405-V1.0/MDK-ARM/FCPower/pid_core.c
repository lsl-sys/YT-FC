#include "pid_core.h"

static inline float Absf(float x) 
{ 
    return (x < 0.0f) ? -x : x; 
}

void PID_Init(PIDController* pid, const float desired, const pidParam_t pidParam)
{
    if (pid == NULL) {
        return; 
    }
    
    pid->desired = desired;
    pid->measure = 0.0f;  // 初始测量值设为0
		pid->prevMeasure = 0.0f;
    
    pid->error = 0.0f;    // 当前误差
    
    pid->kp = pidParam.kp;
    pid->ki = pidParam.ki;
    pid->kd = pidParam.kd;
		pid->iSepThresh = pidParam.iSepThresh;
    
    pid->outP = 0.0f; 
    pid->outI = 0.0f; 
    pid->outD = 0.0f;  
    pid->out = 0.0f;  
    
    pid->integ = 0.0f;  // 积分累积值
    pid->deriv = 0.0f;  // 微分变化率
    
    pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;  // 积分限幅
    pid->outputLimit = DEFAULT_PID_OUTPUT_LIMIT;   // 输出限幅
    pid->deadBand = DEFAULT_PID_DEAD_BAND; // 死区范围，默认为0
    pid->maxErr = DEFAULT_PID_MAX_ERR;   // 最大误差限幅，0表示无限制
		
		pid->firstUpdate = 1; // 标记为首次调用
}

void PID_Reset(PIDController* pid)
{
    if (pid == NULL) {
        return;
    }
    
    pid->error = 0.0f;
		pid->measure = 0.0f;
    pid->prevMeasure = 0.0f;
    
    pid->outP = 0.0f;
    pid->outI = 0.0f;
    pid->outD = 0.0f;
    pid->out = 0.0f;
    
    pid->integ = 0.0f;
    pid->deriv = 0.0f;
		
		pid->firstUpdate = 1;  // 重置首次调用标志
}

float PID_Calculate(PIDController* pid, float _measure, float _target)
{
    if (pid == NULL) {
        return 0.0f;  
    }
    
    pid->desired = _target;
    pid->measure = _measure;
    
    // 计算当前误差
    float error = pid->desired - pid->measure;
		
		// 首次调用保护,避免初始微分冲击
    if (pid->firstUpdate) {
        pid->prevMeasure = _measure;  // 初始化历史测量值
        pid->firstUpdate = 0;
    }
    
    // 死区处理
    if (pid->deadBand > 0.0f && error > -pid->deadBand && error < pid->deadBand)
    {
			  pid->prevMeasure = _measure;
        return pid->out;  // 返回上一次输出，避免频繁切换
    }
    
    // 误差限幅
    if (pid->maxErr > 0.0f)
    {
        if (error > pid->maxErr)
        {
            error = pid->maxErr;
        }
        else if (error < -pid->maxErr)
        {
            error = -pid->maxErr;
        }
    }
    
    // 更新误差变量
    pid->error = error;
    
    // 积分分离：大误差时不累积积分，防止饱和
    if (pid->iSepThresh <= 0.0f || Absf(error) < pid->iSepThresh)// 如果误差大于阈值且阈值>0，则跳过积分更新（保持原值）
    {
        pid->integ += pid->error;
        
        // 积分限幅
        if (pid->integ > pid->iLimit)
            pid->integ = pid->iLimit;
        else if (pid->integ < -pid->iLimit)
            pid->integ = -pid->iLimit;
    }
    
		/** author : lsl-sys*/
    // 计算微分项
    pid->deriv = pid->prevMeasure - pid->measure;
    
    // 计算各输出项
    pid->outP = pid->kp * pid->error;  
    pid->outI = pid->ki * pid->integ; 
    pid->outD = pid->kd * pid->deriv;  
    
    // 计算总输出
    float output = pid->outP + pid->outI + pid->outD;
    
    // 输出限幅
    if (pid->outputLimit > 0.0f)
    {
        if (output > pid->outputLimit)
        {
            output = pid->outputLimit;
        }
        else if (output < -pid->outputLimit)
        {
            output = -pid->outputLimit;
        }
    }
    
    // 更新
     pid->prevMeasure = pid->measure;
    pid->out = output;
    
    return output; 
}

void PID_SetIntegralLimit(PIDController* pid, const float limit)
{
    if (pid != NULL) {
        pid->iLimit = Absf(limit);  // 确保限幅为正数
    }
}

void PID_SetOutputLimit(PIDController* pid, const float limit)
{
    if (pid != NULL) {
        pid->outputLimit = Absf(limit);  // 确保限幅为正数
    }
}

void PID_SetDesired(PIDController* pid, const float desired)
{
    if (pid != NULL) {
        pid->desired = desired;
    }
}

void PID_SetKp(PIDController* pid, const float kp)
{
    if (pid != NULL) {
        pid->kp = kp;
    }
}

void PID_SetKi(PIDController* pid, const float ki)
{
    if (pid != NULL) {
        pid->ki = ki;
    }
}

void PID_SetKd(PIDController* pid, const float kd)
{
    if (pid != NULL) {
        pid->kd = kd;
    }
}

void PID_SetDeadBand(PIDController* pid, const float deadBand)
{
    if (pid != NULL && deadBand > 0.0f) {
        pid->deadBand = deadBand;
    }
}

void PID_SetMaxErr(PIDController* pid, const float maxErr)
{
    if (pid != NULL && maxErr > 0.0f) {
        pid->maxErr = maxErr;
    }
}

void PID_SetISepThresh(PIDController* pid, const float thresh)
{
    if (pid != NULL && thresh >= 0.0f) {
        pid->iSepThresh = thresh;
    }
}
