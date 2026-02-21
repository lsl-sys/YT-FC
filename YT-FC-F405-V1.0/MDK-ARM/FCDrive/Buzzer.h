/**
 * @file       Buzzer.h
 * @author	   lsl-sys
 * @brief      Buzzer driver
 * @version    V1.0.0
 * @date       2026-1-30 
 * @Encoding   UTF-8
 */

#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"   

/**
 * @brief PWM硬件配置说明
 * PWM 频率 2800Hz
 */

/* 蜂鸣器谐振频率(2.7kHz附近声压最大) */
#define BUZZER_RESONANT_HZ  2731U       

typedef struct {
    TIM_HandleTypeDef *htim;   
    uint32_t Channel;          
} Buzzer_HandleTypeDef;

void Buzzer_init(Buzzer_HandleTypeDef *buzz);  // 初始化并启动PWM
void Buzzer_SetTone(Buzzer_HandleTypeDef *buzz, uint16_t freq_hz);  // 改音调(Hz),0=静音
void Buzzer_SetVolume(Buzzer_HandleTypeDef *buzz, uint8_t vol);     // 调音量(0-100),不改音调
void Buzzer_Beep(Buzzer_HandleTypeDef *buzz, uint16_t ms);          // 短鸣一声(ms,阻塞)
void Buzzer_DoubleBeep(Buzzer_HandleTypeDef *buzz);                 // 双鸣提示(系统就绪,阻塞)

#endif 
