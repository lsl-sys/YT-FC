#include "imu.h"

imu_data_t imu = {0};

/* 量程限制 */
#define ANGLE_MIN   -180.0f
#define ANGLE_MAX    180.0f
#define GYRO_MIN   -2000.0f
#define GYRO_MAX    2000.0f

/**
 * @brief  数据范围有效性检查
 * @param  v 被检查值
 * @param  min 最小值
 * @param  max 最大值
 * @return 1-有效，0-超界
 */
static inline uint8_t in_range(float v, float min, float max)
{
    return (v >= min && v <= max);
}

#if IMU_MODE == 1
/**
 * @brief  一阶低通滤波器（针对电机抖动优化）
 * @param  new_val 新采样值
 * @param  last_val 上次滤波值
 * @param  alpha 滤波系数（IMU_FILTER_ALPHA）
 * @return 滤波后值
 * @note   公式：y(n) = alpha * x(n) + (1-alpha) * y(n-1)
 *         电机抖动通常为高频（>100Hz），低通可有效抑制
 */
static inline float lowpass_filter(float new_val, float last_val, float alpha)
{
    return alpha * new_val + (1.0f - alpha) * last_val;
}

/** @brief 滤波历史值（静态变量保持状态） */
static struct {
    float roll, pitch, yaw;
    float gx, gy, gz;
} filter_hist = {0};
#endif

void imu_init(void)
{
    memset(&imu, 0, sizeof(imu_data_t));
    wt901c_init();
    
#if IMU_MODE == 1
    memset(&filter_hist, 0, sizeof(filter_hist));
#endif
}

void imu_reset(void)
{
#if IMU_MODE == 1
    // 重置滤波器历史值为当前传感器值，避免跳变
    filter_hist.roll = wt901c_data.roll;
    filter_hist.pitch = wt901c_data.pitch;
    filter_hist.yaw = wt901c_data.yaw;
    filter_hist.gx = wt901c_data.wx;
    filter_hist.gy = wt901c_data.wy;
    filter_hist.gz = wt901c_data.wz;
#endif
}

/**
 * @brief  IMU 数据更新主函数
 * @note   根据 IMU_MODE 宏定义选择工作模式：
 *         Mode 0：直接透传，仅做范围检查
 *         Mode 1：低通滤波，抑制电机抖动（推荐用于飞行器）
 */
void imu_update(void)
{

    
    // 继承在线状态
    imu.online = wt901c_data.online;
    
    // 离线或无效时直接返回（保持上次值或清零，根据需求）
    if (!imu.online) {
        imu.valid = 0;
        return;
    }
    /** author : lsl-sys*/
    // 范围检查（两种模式都执行）
    if (!in_range(wt901c_data.roll, ANGLE_MIN, ANGLE_MAX) ||
        !in_range(wt901c_data.pitch, ANGLE_MIN, ANGLE_MAX) ||
        !in_range(wt901c_data.wx, GYRO_MIN, GYRO_MAX)) {
        imu.valid = 0;
        return;
    }
    imu.valid = 1;
    
#if IMU_MODE == 0
    /* ============== 模式 0：简单透传 ============== */
    // 直接复制，无计算开销
    imu.roll  = wt901c_data.roll;
    imu.pitch = wt901c_data.pitch;
    imu.yaw   = wt901c_data.yaw;
    
    imu.gx = wt901c_data.wx;
    imu.gy = wt901c_data.wy;
    imu.gz = wt901c_data.wz;
    
    imu.ax = wt901c_data.ax;  // 如 WT901C 未解析加速度，可删除
    imu.ay = wt901c_data.ay;
    imu.az = wt901c_data.az;
    
#else
    /* ============== 模式 1：抖动抑制滤波 ============== */
    // 角度滤波（必须）：电机抖动直接影响姿态角，需深度平滑
    // alpha=0.25 含义：新值占25%，历史值占75%，能有效抑制 100Hz 级电机振动
    imu.roll  = lowpass_filter(wt901c_data.roll,  filter_hist.roll,  IMU_FILTER_ALPHA);
    imu.pitch = lowpass_filter(wt901c_data.pitch, filter_hist.pitch, IMU_FILTER_ALPHA);
    imu.yaw   = lowpass_filter(wt901c_data.yaw,   filter_hist.yaw,   IMU_FILTER_ALPHA);
    
    // 角速度滤波（轻度）：用于 PID 微分项时，高频噪声会放大，建议轻度滤波
    // 注意：若用于姿态解算积分，建议 alpha 更大（如 0.5）或不过滤，避免相位延迟
    #define GYRO_ALPHA  0.4f  // 角速度滤波系数可独立设置，通常比角度大（响应更快）
    imu.gx = lowpass_filter(wt901c_data.wx, filter_hist.gx, GYRO_ALPHA);
    imu.gy = lowpass_filter(wt901c_data.wy, filter_hist.gy, GYRO_ALPHA);
    imu.gz = lowpass_filter(wt901c_data.wz, filter_hist.gz, GYRO_ALPHA);
    
    
    // 更新历史值
    filter_hist.roll  = imu.roll;
    filter_hist.pitch = imu.pitch;
    filter_hist.yaw   = imu.yaw;
    filter_hist.gx    = imu.gx;
    filter_hist.gy    = imu.gy;
    filter_hist.gz    = imu.gz;
#endif
}

