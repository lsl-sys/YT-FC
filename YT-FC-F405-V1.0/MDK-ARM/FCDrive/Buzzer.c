#include "Buzzer.h"

#define TIM_CLK_HZ              56000000U

/* 短鸣默认音量 (0-100%) */
#define BUZZER_BEEP_VOLUME      75

void Buzzer_init(Buzzer_HandleTypeDef *buzz)
{
    __HAL_TIM_SET_COMPARE(buzz->htim, buzz->Channel, 0);
    HAL_TIM_PWM_Start(buzz->htim, buzz->Channel);
}

void Buzzer_SetTone(Buzzer_HandleTypeDef *buzz, uint16_t freq_hz)
{
    if (freq_hz == 0) {
        __HAL_TIM_SET_COMPARE(buzz->htim, buzz->Channel, 0);
        return;
    }
    
    /* 计算新ARR: 56MHz / freq - 1 */
    uint32_t new_arr = (TIM_CLK_HZ / freq_hz) - 1;
    if (new_arr > 65535) new_arr = 65535;
    
    /* 保持占空比比例, 防止变调时音量突变 */
    uint32_t old_ccr = __HAL_TIM_GET_COMPARE(buzz->htim, buzz->Channel);
    uint32_t old_arr = __HAL_TIM_GET_AUTORELOAD(buzz->htim);
    
    __HAL_TIM_SET_AUTORELOAD(buzz->htim, new_arr);
    
    if (old_arr > 0) {
        uint32_t new_ccr = (old_ccr * new_arr) / old_arr;
        __HAL_TIM_SET_COMPARE(buzz->htim, buzz->Channel, new_ccr);
    }
}

void Buzzer_SetVolume(Buzzer_HandleTypeDef *buzz, uint8_t vol)
{
    if (vol > 100) vol = 100;
    
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(buzz->htim);
    __HAL_TIM_SET_COMPARE(buzz->htim, buzz->Channel, (arr * vol) / 100);
}

void Buzzer_Beep(Buzzer_HandleTypeDef *buzz, uint16_t ms)
{
    Buzzer_SetVolume(buzz, BUZZER_BEEP_VOLUME);
    HAL_Delay(ms);
    Buzzer_SetVolume(buzz, 0);
}

void Buzzer_DoubleBeep(Buzzer_HandleTypeDef *buzz)
{
    Buzzer_Beep(buzz, 100);  // 嘀
    HAL_Delay(100);          // 间隔
    Buzzer_Beep(buzz, 100);  // 嘀
}
