#include "WT901C.h"

#define WT901C_HUART        huart3
#define WT901C_DMA_INSTANCE hdma_usart3_rx
extern DMA_HandleTypeDef hdma_usart3_rx;

#define WT901C_HEADER       0x55
#define WT901C_FRAME_LEN    11          // Header+Type+Data(8)+Sum

#define TYPE_ACC            0x51
#define TYPE_GYRO           0x52
#define TYPE_ANGLE          0x53
#define TYPE_MAG            0x54

#define BUF_SIZE            32

/*小端模式合成 16 位有符号数*/
#define INT16_FROM_BYTES(l, h) ((int16_t)((uint8_t)(h) << 8 | (uint8_t)(l)))

bool wt901c_flag_of_receive = 0;
uint8_t wt901c_rx_buf[BUF_SIZE];
uint8_t wt901c_frame[WT901C_FRAME_LEN];

wt901c wt901c_data;
wt901c_raw_data wt901c_data_raw;

/*用于超时检测，判断传感器是否在线*/
static uint32_t wt901c_last_tick = 0;

/*校验和计算（前10字节累加取低8位）*/
static uint8_t calc_checksum(const uint8_t *p)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 10; i++) sum += p[i];
    return sum;
}

/*解析角速度帧（0x52）*/
static void parse_gyro(const uint8_t *d)
{
    wt901c_data_raw.WxL = d[0]; wt901c_data_raw.WxH = d[1];
    wt901c_data_raw.WyL = d[2]; wt901c_data_raw.WyH = d[3];
    wt901c_data_raw.WzL = d[4]; wt901c_data_raw.WzH = d[5];
    
    wt901c_data.wx = INT16_FROM_BYTES(d[0], d[1]) / 32768.0f * 2000.0f;
    wt901c_data.wy = INT16_FROM_BYTES(d[2], d[3]) / 32768.0f * 2000.0f;
    wt901c_data.wz = INT16_FROM_BYTES(d[4], d[5]) / 32768.0f * 2000.0f;
}

/*解析角度帧（0x53）*/
static void parse_angle(const uint8_t *d)
{
    wt901c_data_raw.RollL = d[0];  wt901c_data_raw.RollH = d[1];
    wt901c_data_raw.PitchL = d[2]; wt901c_data_raw.PitchH = d[3];
    wt901c_data_raw.YawL = d[4];   wt901c_data_raw.YawH = d[5];
    
    wt901c_data.roll  = INT16_FROM_BYTES(d[0], d[1]) / 32768.0f * 180.0f;
    wt901c_data.pitch = INT16_FROM_BYTES(d[2], d[3]) / 32768.0f * 180.0f;
    wt901c_data.yaw   = INT16_FROM_BYTES(d[4], d[5]) / 32768.0f * 180.0f;
}

void wt901c_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&WT901C_HUART, wt901c_rx_buf, BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&WT901C_DMA_INSTANCE, DMA_IT_HT);
    memset(wt901c_rx_buf, 0, BUF_SIZE);
    
    // 初始化时设为离线状态，等待首次数据
    wt901c_data.online = 0;
    wt901c_last_tick = 0;
}


void wt901c_analysis_data(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t valid_frame_found = 0;
    
    // 检查超时：若超过 WT901C_TIMEOUT_MS 未收到有效数据，标记为离线
    if (now - wt901c_last_tick > WT901C_TIMEOUT_MS) {
        wt901c_data.online = 0;
    }
    
    // 无新数据，直接返回（已更新在线状态）
    if (!wt901c_flag_of_receive) return;
    wt901c_flag_of_receive = 0;
    
    // 在缓冲区中查找所有可能的帧头（0x55）
    for (uint8_t i = 0; i <= BUF_SIZE - WT901C_FRAME_LEN; i++) {
        if (wt901c_rx_buf[i] != WT901C_HEADER) continue;
        
        uint8_t *p = &wt901c_rx_buf[i];
        if (calc_checksum(p) != p[10]) continue;  // 校验失败跳过
        
        uint8_t type = p[1];
        const uint8_t *payload = &p[2];
        
        // 根据帧类型解析数据
        switch (type) {
            case TYPE_GYRO:  
                parse_gyro(payload);  
                valid_frame_found = 1;
                break;
            case TYPE_ANGLE: 
                parse_angle(payload); 
                valid_frame_found = 1;
                break;
            case TYPE_ACC:   
                /* parse_acc(payload); */  
                valid_frame_found = 1;  // 即使未解析也标记为收到数据
                break;
            case TYPE_MAG:   
                /* parse_mag(payload); */  
                valid_frame_found = 1;
                break;
        }
    }
    
    // 只要解析到任意有效帧，更新在线状态和时间戳
    if (valid_frame_found) {
        wt901c_data.online = 1;
        wt901c_last_tick = now;
    }
    
    // 清空接收缓冲区，准备下次接收/*author : lsl-sys*/
    memset(wt901c_rx_buf, 0, BUF_SIZE);
}


void wt901c_receive_data(void)
{
    wt901c_flag_of_receive = 1;
    HAL_UARTEx_ReceiveToIdle_DMA(&WT901C_HUART, wt901c_rx_buf, BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&WT901C_DMA_INSTANCE, DMA_IT_HT);
}
