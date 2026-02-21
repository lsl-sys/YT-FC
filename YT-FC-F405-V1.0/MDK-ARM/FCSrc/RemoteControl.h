/**
 * @file       RemoteControl.h
 * @author	   lsl-sys
 * @brief      遥控器数据滤波
 * @version    V1.1.0
 * @date       2026-1-11 2026-2-12
 * @Encoding   UTF-8
 */
#ifndef __REMOTECONTROL_H
#define __REMOTECONTROL_H

#include "main.h"
#include "ELRS.h"

/**
 * @brief 通道特性配置宏定义
 */
#define RC_CHANNEL_MIN (-100)                     /* 遥控器通道最小值 */
#define RC_CHANNEL_MAX 100                        /* 遥控器通道最大值 */
#define CH5_8_PRESSED_THRESHOLD 90                /* CH5-CH8按钮按下阈值 */
#define CH9_PRESSED_THRESHOLD 90                  /*CH9自恢复按钮按下阈值 */
#define THREE_STATE_VALID_VALUES { -100, 0, 100 } /*三状态开关有效取值 */
#define ROLLER_SPIKE_THRESHOLD -30                /* 滚轮通道尖峰检测阈值 */

/**
 * @brief 滤波后遥控器数据结构体
 * 包含10个通道的滤波后数据，带通道命名
 */
typedef struct {
    float RX;    /*!< 右摇杆x轴（横滚） */
    float RY;    /*!< 右摇杆y轴（俯仰） */
    float LY;    /*!< 左摇杆y轴（油门） */
    float LX;    /*!< 左摇杆x轴（偏航） */
    float SA;    /*!< 按钮 - 按下100(98左右)，抬起-100 */
    float SB;    /*!< 三状态开关 - -100, 0, 100 */
    float SC;    /*!< 三状态开关 - -100, 0, 100 */
    float SD;    /*!< 按钮 - 按下100(98左右)，抬起-100 */
    float SE;    /*!< 自恢复按钮 - 按下100，抬起-100 */
    float SL;    /*!< 滚轮 - -100到100 */
} rc_channels;

void rc_init(void);

/**
 * @brief 遥控器数据滤波处理函数
 * @param 无
 * @retval 无
 * @note 在1ms中断中调用，处理10个通道的异常值
 */
void rc_filter_process(void);

/**
 * @brief 全局变量声明
 */
extern rc_channels filtered_rc;  /*!< 滤波后的遥控器数据 */


#endif

