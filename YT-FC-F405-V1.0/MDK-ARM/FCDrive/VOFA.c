#include "VOFA.h"

/* 硬件配置：UART8 + DMA 空闲中断，FireWater文本协议 */
#define VOFA_USART_INSTANCE  huart1
#define VOFA_DMA_INSTANCE  hdma_usart1_rx

vofa_var_t var_table[VOFA_MAX_VARS];
uint8_t    var_count = 0;

extern DMA_HandleTypeDef hdma_usart1_rx;

#define VOFA_MAX_RECEIVE_SIZE 128    // 单帧最大长度

/* 双缓冲机制：
   rxBuffer - DMA直接写入，中断中仅做拷贝
   rxData   - 供主循环解析，避免DMA覆盖风险 */
uint8_t vofa_rxBuffer[VOFA_MAX_RECEIVE_SIZE] = {0};
uint8_t vofa_rxData[VOFA_MAX_RECEIVE_SIZE] = {0};

bool vofa_flag_of_receive = 0;       // 帧接收完成标志，解析后自动清零

/* 初始化：启动DMA空闲中断，关闭半传输中断 */
void vofa_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&VOFA_USART_INSTANCE, vofa_rxBuffer, VOFA_MAX_RECEIVE_SIZE);
    __HAL_DMA_DISABLE_IT(&VOFA_DMA_INSTANCE, DMA_IT_HT);
	  memset(vofa_rxBuffer, 0, sizeof(vofa_rxBuffer));
}

/* 中断回调内调用：保存数据到解析区，重启DMA接收 */
void vofa_receive_data(void)
{
	  HAL_UARTEx_ReceiveToIdle_DMA(&VOFA_USART_INSTANCE, vofa_rxBuffer, VOFA_MAX_RECEIVE_SIZE);
    __HAL_DMA_DISABLE_IT(&VOFA_DMA_INSTANCE, DMA_IT_HT);
	  vofa_flag_of_receive = 1;
}

/* 主循环调用：解析"name:value"格式指令，自动匹配已注册变量并赋值
   支持类型：int/float/double/bool，类型不匹配会截断或溢出 */
void vofa_analysis_data(void)
{
    if (!vofa_flag_of_receive) return;
    vofa_flag_of_receive = 0;
	
    memcpy(vofa_rxData, vofa_rxBuffer, VOFA_MAX_RECEIVE_SIZE);
	
    char* buf = (char*)vofa_rxData;
	  char* eq  = strchr(buf, ':');    // FireWater协议分隔符为冒号
    
    if (eq) {
        *eq = '\0';                  // 截断字符串：name\0value
        char* name  = buf;
        char* value = eq + 1;
        
        for (int i = 0; i < var_count; i++) {
            if (strcmp(var_table[i].name, name) == 0) {
                switch (var_table[i].type) {
                    case TYPE_INT:    *(int*)var_table[i].addr = atoi(value); break;
                    case TYPE_FLOAT:  *(float*)var_table[i].addr = atof(value); break;
                    case TYPE_DOUBLE: *(double*)var_table[i].addr = strtod(value, NULL); break;
                    case TYPE_BOOL:   *(bool*)var_table[i].addr = (atoi(value) != 0); break;
                }
                break;               // 匹配成功即退出，同名变量以先注册为准
            }
        }
    }
    
    memset(vofa_rxData, 0, VOFA_MAX_RECEIVE_SIZE);
}
/** author : lsl-sys*/
/* 注册全局变量到Dictionary，供上位机调参使用
   例：float kp; vofa_login_name("kp", &kp, TYPE_FLOAT); */
void vofa_login_name(const char* name, void* addr, vofa_type_t type)
{
    if (var_count >= VOFA_MAX_VARS) return;
    
    var_table[var_count].name = name;
    var_table[var_count].addr = addr;
    var_table[var_count].type = type;
    var_count++;
}

/* 查询接收标志，用于主循环判断是否需处理新数据 */
bool vofa_get_flag_of_receive(void)
{
	return vofa_flag_of_receive;
}

/* 按变量名读取当前值，用于上传数据到上位机显示
   未注册变量返回0.0，建议先确认变量存在再调用 */
double vofa_get_data(const char* name)
{
    for (int i = 0; i < var_count; i++) {
        if (strcmp(var_table[i].name, name) == 0) {
            switch (var_table[i].type) {
                case TYPE_INT:    return *(int*)var_table[i].addr;
                case TYPE_FLOAT:  return *(float*)var_table[i].addr;
                case TYPE_DOUBLE: return *(double*)var_table[i].addr;
                case TYPE_BOOL:   return *(bool*)var_table[i].addr ? 1.0 : 0.0;
            }
        }
    }
    return 0.0;
}
