/**
 * @file       OpticalFlow.c
 * @brief      T1-001plus 光流解算与传感器掉线保护
 * @note       关键保护：传感器拔下或超时后，立即停止积分，速度清零
 *             防止残留数据导致位移持续漂移
 */

#include "OpticalFlow.h"
#include "math.h"

optical_flow_t optflow = {0};

#define OFFLINE_TIMEOUT     200     // 200ms无数据视为离线
#define HEIGHT_MIN          50      
#define HEIGHT_MAX          4000    
#define QUALITY_MIN         30      
#define STATIC_THRESHOLD    10      

#define HEIGHT_DELTA_MAX    150     
#define LOWPASS_ALPHA       0.15f   

static uint32_t last_tick = 0;
static float height_last = 0;       

/**
 * @brief  中值滤波（3点窗口）
 */
static float median_filter_3(float new_val)
{
    static float buf[3] = {0};
    static uint8_t idx = 0;
    
    buf[idx] = new_val;
    idx = (idx + 1) % 3;
    
    float a = buf[0], b = buf[1], c = buf[2];
    float temp;
    
    if (a > b) { temp = a; a = b; b = temp; }
    if (b > c) { temp = b; b = c; c = temp; }
    if (a > b) { temp = a; a = b; b = temp; }
    
    return b;  
}

/**
 * @brief  高度三级滤波：中值 -> 限幅 -> 低通
 */
static float filter_height(float raw_height)
{
    float median = median_filter_3(raw_height);
    
    float delta = median - height_last;
    if (fabsf(delta) > HEIGHT_DELTA_MAX) {
        return height_last;  
    }
    
    float filtered = LOWPASS_ALPHA * median + (1.0f - LOWPASS_ALPHA) * height_last;
    height_last = filtered;
    return filtered;
}

/**
 * @brief  计算运动学参数（仅在数据有效时调用）
 */
static void calculate_motion(const t1plus *raw)
{
    float dt_s = raw->integration_timespan / 1000000.0f;
    if (dt_s < 0.001f) dt_s = 0.01f;
    
    float dx = raw->actual_flow_x;
    float dy = raw->actual_flow_y;
    
    optflow.vel_x = dx / dt_s;
    optflow.vel_y = dy / dt_s;
    
    optflow.pos_x += dx;
    optflow.pos_y += dy;
    
    float speed = sqrtf(optflow.vel_x * optflow.vel_x + optflow.vel_y * optflow.vel_y);
    optflow.is_moving = (speed > STATIC_THRESHOLD) ? 1 : 0;
}

void optical_flow_init(void)
{
    memset(&optflow, 0, sizeof(optical_flow_t));
    T1Plus_init();
    last_tick = 0;
    height_last = 0;
}

void optical_flow_reset(void)
{
    optflow.pos_x = 0.0f;
    optflow.pos_y = 0.0f;
    height_last = optflow.height_filtered;
}
/** author : lsl-sys*/
/**
 * @brief  光流数据更新（带掉线保护）
 * @note   修复：传感器拔下后（online=0），立即停止积分并清零速度
 */
void optical_flow_update(void)
{
    uint32_t now = HAL_GetTick();
    const t1plus *raw = &t1plus_data;
    
    /* ========== 步骤1：超时检测（掉线保护）========== */
    if (now - last_tick > OFFLINE_TIMEOUT) {
        // 刚掉线时清零状态
        if (optflow.online) {
            optflow.online = 0;
            optflow.valid = 0;
            // 关键修复：清零速度，防止用旧速度继续积分
            optflow.vel_x = 0;
            optflow.vel_y = 0;
            optflow.is_moving = 0;
        }
        // 已处于离线状态，直接返回，不再积分
        return;
    }
    
    /* ========== 步骤2：T1Plus数据有效性检查 ========== */
    if (raw->valid != T1PLUS_VALID_DATA) {
        // 数据标志异常（如拔下后残留旧数据但无新帧）
        optflow.valid = 0;
        // 同样清零速度，防止积分漂移
        optflow.vel_x = 0;
        optflow.vel_y = 0;
        optflow.is_moving = 0;
        // 注意：此时online仍为1（未超时），但数据无效
        return;
    }
    
    /* ========== 步骤3：物理约束检查 ========== */
    float raw_height = (float)raw->laser_distance;
    if (raw_height < HEIGHT_MIN || raw_height > HEIGHT_MAX ||
        raw->laser_confidence < QUALITY_MIN) {
        optflow.valid = 0;
        optflow.online = 1;  
        // 数据超界时也应停止积分（或根据需求决定是否使用）
        optflow.vel_x = 0;
        optflow.vel_y = 0;
        return;
    }
    
    /* ========== 步骤4：正常数据更新 ========== */
    optflow.online = 1;
    optflow.valid = 1;
    optflow.quality = raw->laser_confidence;
    optflow.height = raw_height;
    optflow.height_filtered = filter_height(raw_height);
    
    // 只有数据完全有效时才计算运动和积分
    calculate_motion(raw);
    
    // 更新时间戳（关键：只有成功处理有效数据才刷新时间戳）
    last_tick = now;
}

void optical_flow_print(void)
{
    printf("[OpticalFlow]:%d,%d,%.1f,%.1f,%d,%.2f,%.2f,%.2f,%.2f,%d\r\n",
           optflow.online,
           optflow.valid,
           optflow.height,
           optflow.height_filtered,
           optflow.quality,
           optflow.vel_x,
           optflow.vel_y,
           optflow.pos_x,
           optflow.pos_y,
           optflow.is_moving
    );
}
