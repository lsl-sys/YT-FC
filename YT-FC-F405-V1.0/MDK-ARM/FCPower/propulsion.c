#include "propulsion.h"
#include "pid_control.h"
#include "stdio.h"

static MotorHandle_t g_motors[MOTOR_COUNT]; // 保存句柄
static uint8_t       g_ready = 0;           // 就绪标志

/* 限幅 */
static inline float Constrain(float val, float min, float max) {
    return (val < min) ? min : ((val > max) ? max : val);
}

/* 将0-100速度映射到CCR寄存器值 */
static inline uint32_t SpeedToCCR(float speed) {
    return (uint32_t)(PWM_MIN_COMPARE + speed / 100.0f * (PWM_MAX_COMPARE - PWM_MIN_COMPARE));
}

void Propulsion_Init(const MotorHandle_t motors[MOTOR_COUNT]) {
    // 保存句柄
    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_motors[i] = motors[i];
    }
    
    // 启动PWM
    for (int i = 0; i < MOTOR_COUNT; i++) {
        HAL_TIM_PWM_Start(g_motors[i].htim, g_motors[i].channel);
    }
		
    // 解锁电调：最小油门保持2秒
    for (int i = 0; i < MOTOR_COUNT; i++) {
        __HAL_TIM_SET_COMPARE(g_motors[i].htim, g_motors[i].channel, PWM_MIN_COMPARE);
    }
    
    g_ready = 1;
}

/* 混控并输出 author : lsl-sys*/ 
void Propulsion_MixOutput(float throttle, float pitch, float roll, float yaw) {
//	printf("g_ready:%d",g_ready);
    if (!g_ready) return;
    
    // 安全保护
    if (g_pid.fault || !g_pid.arm_flag) {
        Propulsion_Stop();
        return;
    }
    
//		printf("pry:%f,%f,%f.%f\r\n",pitch,roll,yaw,throttle);
		float m[MOTOR_COUNT];
    m[MOTOR_FL] = throttle + pitch + roll + yaw;   // M1 前左
    m[MOTOR_FR] = throttle - pitch + roll - yaw;   // M2 前右
    m[MOTOR_BR] = throttle - pitch - roll + yaw;   // M3 后右
    m[MOTOR_BL] = throttle + pitch - roll - yaw;   // M4 后左
    
    // 限幅并输出
    for (int i = 0; i < MOTOR_COUNT; i++) {
        m[i] = Constrain(m[i], MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT);
        __HAL_TIM_SET_COMPARE(g_motors[i].htim, g_motors[i].channel, SpeedToCCR(m[i]));
    }
}

/* 单独设置单个电机（调试用， bypass 混控和保护） */
void Propulsion_SetSingle(MotorID_t id, float speed) {
    if (!g_ready || id >= MOTOR_COUNT) return;
    
    speed = Constrain(speed, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT);
	
//	  printf("%d\r\n",SpeedToCCR(speed));
    __HAL_TIM_SET_COMPARE(g_motors[id].htim, g_motors[id].channel, SpeedToCCR(speed));
}

/* 紧急停止 */
void Propulsion_Stop(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        __HAL_TIM_SET_COMPARE(g_motors[i].htim, g_motors[i].channel, PWM_MIN_COMPARE);
    }
//    g_ready = 0;
}

/* 检查就绪状态（软件层面） */
uint8_t Propulsion_IsReady(void) {
    return (g_ready && g_pid.arm_flag && !g_pid.fault);
}
