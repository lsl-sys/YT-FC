#include "Interrupt.h"
#include "Scheduler.h"

extern vofa_pid_value vofa_pid;

//UART中断回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart->Instance == USART1)
  {
		vofa_receive_data();
  }
	else if(huart->Instance == USART6)
  {
		elrs_receive_data();
  }
	else if(huart->Instance == USART3)
  {
		wt901c_receive_data();
  }
	else if(huart->Instance == USART2)
  {
		T1Plus_receive_data();
  }
}

//TIM中断回调函数 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//2.5ms
{ 
    static uint32_t timer_counter = 0; 
    if(htim->Instance == TIM1)
    { 
			 
        
        timer_counter++; 
        if(timer_counter % 5 == 0) 
        { 
            
        } 
        else if(timer_counter >= 200)
        { 
            timer_counter = 0; 
        } 
    } 
}
 
