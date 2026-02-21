/**
 * @file       ELRS.h
 * @author	   lsl-sys
 * @brief      RC Receiver Driver (ELRS Protocol) Using USART DMA IDLE Line Detection
 * @version    V2.3.0
 * @date       2025-12-13 2026-1-30 2026-2-13
 * @Encoding   UTF-8
 */
 
#ifndef __ELRS_H
#define __ELRS_H

#include "main.h"
#include "ELRS_Def.h"

/**
 * @brief ELRS 串口配置
 * 波特率: 420000
 * @brief SBUS 串口配置
 * 波特率: 100000
 */


/* ================= 配置区域 ================= */
#define ELRS_USE_CRSF       1           // 1: 使用CRSF协议, 0: 使用SBUS协议

#define ELRS_PACKET_RATE    250         // 刷新率: 50/150/250 Hz

//允许连续丢失的帧数 
#define ELRS_LOST_TOLERANCE 10        

#if ELRS_PACKET_RATE == 250
    #define ELRS_TIMEOUT_MS (4 * (ELRS_LOST_TOLERANCE + 1))   
#elif ELRS_PACKET_RATE == 150
    #define ELRS_TIMEOUT_MS (7 * (ELRS_LOST_TOLERANCE + 1))  
#elif ELRS_PACKET_RATE == 50
    #define ELRS_TIMEOUT_MS (20 * (ELRS_LOST_TOLERANCE + 1)) 
#else
    #define ELRS_TIMEOUT_MS 30
#endif

#define ELRS_FAILSAFE_MODE  2           // 0:保持最后值 1:归零 2:自定义

#define ELRS_CHAN_NUM       10          // 通道数

typedef struct {
    int8_t ch1, ch2, ch3, ch4;         // 摇杆: 横滚、俯仰、油门、偏航
    int8_t ch5, ch6, ch7, ch8;         // 开关通道
    int8_t ch9, ch10;                  // 辅助
} rc_raw_ch;

typedef struct {
    uint8_t frame_valid;                // 当前帧有效
    uint8_t is_connected;               // 连接状态（经容忍判定后）
    uint8_t lost_count;                 // 连续丢失帧计数
    uint32_t last_tick;                 // 上次有效帧时间戳
} elrs_status_t;

/* ================= 协议选择与接口映射 ================= */
#if ELRS_USE_CRSF

void crsf_init(void);
void crsf_receive_data(void);
void crsf_analysis_data(void);

static inline uint8_t crsf_is_connected(void) { 
    extern elrs_status_t elrs_status;
    return elrs_status.is_connected; 
}
static inline uint8_t crsf_frame_valid(void) { 
    extern elrs_status_t elrs_status;
    return elrs_status.frame_valid; 
}

#define elrs_init           crsf_init
#define elrs_receive_data   crsf_receive_data
#define elrs_analysis_data  crsf_analysis_data
#define elrs_is_connected   crsf_is_connected
#define elrs_frame_valid    crsf_frame_valid

#else

void sbus_init(void);
void sbus_receive_data(void);
void sbus_analysis_data(void);

static inline uint8_t sbus_is_connected(void) { 
    return 0; 
}
static inline uint8_t sbus_frame_valid(void) { 
    return 0; 
}

#define elrs_init           sbus_init
#define elrs_receive_data   sbus_receive_data
#define elrs_analysis_data  sbus_analysis_data
#define elrs_is_connected   sbus_is_connected
#define elrs_frame_valid    sbus_frame_valid

#endif

extern rc_raw_ch rc_raw_channels;
extern elrs_status_t elrs_status;

#endif
