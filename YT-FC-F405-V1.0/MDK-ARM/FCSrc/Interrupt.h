/**
 * @file       IT_Service.h
 * @author	   lsl-sys
 * @brief      Interrupt service routine
 * @version    V1.0.0
 * @date       2025-11-23   
 * @Encoding   UTF-8
 */
#ifndef __IT_SERVICE_H
#define __IT_SERVICE_H

#include "main.h"
#include "usart.h"
#include "stm32f4xx_it.h"

/*UART接收事件中断回调函数*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/*TIM定时器溢出中断回调函数*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif
