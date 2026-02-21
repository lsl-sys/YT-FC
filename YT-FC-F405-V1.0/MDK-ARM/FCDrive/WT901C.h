/**
 * @file       WT901C.h
 * @author	   lsl-sys
 * @brief      WT901C IMU/AHRS Driver with USART DMA IDLE Line Detection
 * @version    V2.2.0
 * @date       2025-11-23  2026-1-30 2026-2-14
 * @Encoding   UTF-8 
 */

#ifndef __WT901C_H
#define __WT901C_H

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "WT901C_Def.h"

/* 通信超时配置: 200ms（约20帧容忍，默认100Hz输出） */
#define WT901C_TIMEOUT_MS   200

/* 初始化DMA空闲中断接收 */
void wt901c_init(void);

/* 接收中断回调: 重启DMA接收（ USART_IRQHandler 中调用）*/
void wt901c_receive_data(void);

/* 数据解析与在线检测（建议10ms周期调用，匹配传感器100Hz输出）*/
void wt901c_analysis_data(void);

/* 获取传感器在线状态: 1-在线, 0-离线（200ms内未收到有效数据）*/
static inline uint8_t wt901c_is_online(void) {
    extern wt901c wt901c_data;
    return wt901c_data.online;
}

extern wt901c wt901c_data;

#endif
