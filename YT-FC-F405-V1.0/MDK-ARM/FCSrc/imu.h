/**
 * @file       imu.h
 * @brief      WT901C 数据继承与姿态滤波层
 * @version    V1.0.0
 * @date       2026-02-14
 * @Encoding   UTF-8 
 */

#ifndef __IMU_H
#define __IMU_H

#include "main.h"
#include "WT901C.h"

/* ================= 工作模式配置 ================= */
#define IMU_MODE            1           // 0:透传模式  1:滤波模式

#if IMU_MODE == 1
    #define IMU_FILTER_ALPHA    0.25f   // 低通系数 0-1，越小越平滑（0.25平衡响应与噪声）
#endif
/* ============================================== */

/** @brief IMU 数据结构 */
typedef struct {
    uint8_t online;          // 传感器在线状态（透传 WT901C）
    uint8_t valid;           // 数据有效标志
    
    float roll;              // 横滚角 °  (-180~180)
    float pitch;             // 俯仰角 °  (-180~180)
    float yaw;               // 偏航角 °  (-180~180)
    
    float gx;                // 角速度 X °/s  (-2000~2000)
    float gy;                // 角速度 Y °/s
    float gz;                // 角速度 Z °/s
    
} imu_data_t;

void imu_init(void);
void imu_update(void);      
void imu_reset(void);        // 重置滤波器状态

extern imu_data_t imu;   

#endif
