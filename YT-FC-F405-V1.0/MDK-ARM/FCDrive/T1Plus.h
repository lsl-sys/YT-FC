/**
 * @file       T1Plus.h
 * @author	   lsl-sys
 * @brief      T1Plus Optical Flow Sensor Driver Using USART DMA IDLE
 * @version    V2.0.0
 * @date       2026-01-11  2026-01-30
 * @Encoding   UTF-8 
 */
#ifndef __T1PLUS_H
#define __T1PLUS_H

#include "main.h"

#define T1PLUS_FRAME_HEADER_1  0xFE        // 帧头
#define T1PLUS_FRAME_LENGTH    0x0A        // 数据包长度
#define T1PLUS_FRAME_TAIL      0x55        // 帧尾
#define T1PLUS_VALID_DATA      0xF5        // 有效数据标识

/* 原始数据帧结构 */
typedef struct {
    uint8_t flow_x_integral_L;      // X位移低字节
    uint8_t flow_x_integral_H;      // X位移高字节
    uint8_t flow_y_integral_L;      // Y位移低字节
    uint8_t flow_y_integral_H;      // Y位移高字节
    uint8_t integration_timespan_L; // 积分时间低字节
    uint8_t integration_timespan_H; // 积分时间高字节
    uint8_t laser_distance_L;       // 测距低字节
    uint8_t laser_distance_H;       // 测距高字节
    uint8_t valid;                  // 数据有效标志(0xF5有效)
    uint8_t laser_confidence;       // 测距置信度(0-100%)
    uint8_t checksum;               // 校验和(字节2-11异或)
} t1plus_raw_data;

/* 解析后数据结构 */
typedef struct {
    int16_t flow_x_integral;        // X像素累计位移 (radians*10000)
    int16_t flow_y_integral;        // Y像素累计位移 (radians*10000)
    uint16_t integration_timespan;  // 积分时间间隔 (us)
    uint16_t laser_distance;        // 激光测距高度 (mm)
    uint8_t valid;                  // 有效性标志
    uint8_t laser_confidence;       // 测距置信度
    float actual_flow_x;            // 实际X位移(mm) = integral/10000 * height
    float actual_flow_y;            // 实际Y位移(mm)
} t1plus;

/** 初始化T1Plus串口DMA接收（开启空闲中断） */
void T1Plus_init(void);

/** 串口接收完成回调 */
void T1Plus_receive_data(void);

/** 数据解析与处理 */
void T1Plus_analysis_data(void);

extern t1plus t1plus_data;

#endif
