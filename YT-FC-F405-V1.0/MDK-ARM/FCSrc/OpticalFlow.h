/**
 * @file       OpticalFlow.h
 * @author	   lsl-sys
 * @brief      T1-001plus Optical Flow Navigation with Height Filtering
 * @version    V1.2.0
 * @date       2026-02-14 2026-02-15
 * @Encoding   UTF-8 
 */

#ifndef __OPTICAL_FLOW_H
#define __OPTICAL_FLOW_H

#include "main.h"
#include "T1Plus.h"

typedef struct {
    uint8_t online;              // 通信状态：1-在线，0-超时
    uint8_t valid;               // 数据有效：1-可用，0-超界/置信度不足
    
    float height;                // 原始激光高度 (mm)
    float height_filtered;       // 滤波后高度 (mm)
    uint8_t quality;             // 数据质量 0-100%
    
    float vel_x;                 // X方向速度 (mm/s)
    float vel_y;                 // Y方向速度 (mm/s)
    
    float pos_x;                 // X方向累计位移 (mm)
    float pos_y;                 // Y方向累计位移 (mm)
    
    uint8_t is_moving;           // 运动状态：1-移动中，0-静止
    
} optical_flow_t;

void optical_flow_init(void);
void optical_flow_update(void);
void optical_flow_reset(void);
void optical_flow_print(void);   // 调试打印

extern optical_flow_t optflow;

#endif
