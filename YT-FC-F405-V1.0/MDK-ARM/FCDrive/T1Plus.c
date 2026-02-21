#include "T1Plus.h"

#define T1Plus_HUART            huart2
#define T1Plus_DMA_INSTANCE     hdma_usart2_rx

extern DMA_HandleTypeDef hdma_usart2_rx;

#define T1Plus_RX_BUFFER_MAXIMUM 24

bool T1Plus_flag_of_receive = 0;
uint8_t t1plus_rx_buf[T1Plus_RX_BUFFER_MAXIMUM];
uint8_t t1plus_rx_data[T1Plus_RX_BUFFER_MAXIMUM];

t1plus t1plus_data;
t1plus_raw_data t1plus_data_raw;

/** 初始化T1Plus DMA接收（配置USART空闲中断，禁用半传输中断） */
void T1Plus_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&T1Plus_HUART, t1plus_rx_buf, T1Plus_RX_BUFFER_MAXIMUM);
    __HAL_DMA_DISABLE_IT(&T1Plus_DMA_INSTANCE, DMA_IT_HT);
    memset(t1plus_rx_buf, 0, sizeof(t1plus_rx_buf));
}

/** 解析T1Plus数据帧（校验帧头、帧尾、校验和，计算实际位移） */
void T1Plus_data_parse(void)
{
    // 计算校验和 (3-12字节异或)
    uint8_t checksum = t1plus_rx_data[2]; // 第3字节
    for(uint8_t i = 3; i < 12; i++) {      // 第4-12字节
        checksum ^= t1plus_rx_data[i];
    }
    
    // 检查帧头、帧长度、校验和和帧尾
    if(t1plus_rx_data[0] == T1PLUS_FRAME_HEADER_1 && 
       t1plus_rx_data[1] == T1PLUS_FRAME_LENGTH && 
       checksum == t1plus_rx_data[12] && 
       t1plus_rx_data[13] == T1PLUS_FRAME_TAIL)
    {
        // 保存原始数据
        t1plus_data_raw.flow_x_integral_L = t1plus_rx_data[2];
        t1plus_data_raw.flow_x_integral_H = t1plus_rx_data[3];
        t1plus_data_raw.flow_y_integral_L = t1plus_rx_data[4];
        t1plus_data_raw.flow_y_integral_H = t1plus_rx_data[5];
        t1plus_data_raw.integration_timespan_L = t1plus_rx_data[6];
        t1plus_data_raw.integration_timespan_H = t1plus_rx_data[7];
        t1plus_data_raw.laser_distance_L = t1plus_rx_data[8];
        t1plus_data_raw.laser_distance_H = t1plus_rx_data[9];
        t1plus_data_raw.valid = t1plus_rx_data[10];
        t1plus_data_raw.laser_confidence = t1plus_rx_data[11];
        t1plus_data_raw.checksum = t1plus_rx_data[12];
        
        // 解析光流X累加位移 (radians*10000)
        t1plus_data.flow_x_integral = (int16_t)((t1plus_data_raw.flow_x_integral_H << 8) | t1plus_data_raw.flow_x_integral_L);
        
        // 解析光流Y累加位移 (radians*10000)
        t1plus_data.flow_y_integral = (int16_t)((t1plus_data_raw.flow_y_integral_H << 8) | t1plus_data_raw.flow_y_integral_L);
        
        // 解析累计时间 (us)
        t1plus_data.integration_timespan = (uint16_t)((t1plus_data_raw.integration_timespan_H << 8) | t1plus_data_raw.integration_timespan_L);
        
        // 解析激光测距距离 (mm)
        t1plus_data.laser_distance = (uint16_t)((t1plus_data_raw.laser_distance_H << 8) | t1plus_data_raw.laser_distance_L);
        
        // 解析状态值
        t1plus_data.valid = t1plus_data_raw.valid;
        
        // 解析激光测距置信度
        t1plus_data.laser_confidence = t1plus_data_raw.laser_confidence;
        
        // 计算实际位移 (mm) = flow_integral / 10000 * height (mm)
        // 注意：这里的height需要从其他传感器获取，如气压计或TOF的高度数据
        // 这里暂时使用激光测距的高度作为示例
        t1plus_data.actual_flow_x = (float)t1plus_data.flow_x_integral / 10000.0f * (float)t1plus_data.laser_distance;
        t1plus_data.actual_flow_y = (float)t1plus_data.flow_y_integral / 10000.0f * (float)t1plus_data.laser_distance;
    }
}

/** 数据解析（滑动窗口查找帧头，拷贝数据并调用解析） */
void T1Plus_analysis_data(void)
{
	  if (!T1Plus_flag_of_receive) return;
    T1Plus_flag_of_receive = 0;
	
    for(uint8_t i = 0; i < T1Plus_RX_BUFFER_MAXIMUM - 13; i++)
    {
        if(t1plus_rx_buf[i] == T1PLUS_FRAME_HEADER_1 && t1plus_rx_buf[i+1] == T1PLUS_FRAME_LENGTH)// 找到有效帧
        {
            memcpy(t1plus_rx_data, &t1plus_rx_buf[i], 14);
            memset(t1plus_rx_buf, 0, sizeof(t1plus_rx_buf));
            break;
        }
    }
	
	  T1Plus_data_parse();
    
}

/** 串口接收完成回调（重启DMA接收，置位接收标志） */
void T1Plus_receive_data(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&T1Plus_HUART, t1plus_rx_buf, T1Plus_RX_BUFFER_MAXIMUM);
    __HAL_DMA_DISABLE_IT(&T1Plus_DMA_INSTANCE, DMA_IT_HT);
		T1Plus_flag_of_receive = 1;
}
