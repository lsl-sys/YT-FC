#include "ELRS.h"

#if ELRS_USE_CRSF

#define ELRS_HUART          huart6
#define ELRS_DMA_INSTANCE   hdma_usart6_rx
extern DMA_HandleTypeDef hdma_usart6_rx;

#define CRSF_SYNC           0xC8
#define CRSF_TYPE_RC        0x16
#define CRSF_PAYLOAD_LEN    22
#define CRSF_FRAME_LEN      26          // 完整帧长度
#define BUF_SIZE            32

#define RC_MIN              172
#define RC_MAX              1811

bool elrs_flag_of_receive = 0;
uint8_t elrs_rx_buf[BUF_SIZE];
uint8_t elrs_payload[CRSF_PAYLOAD_LEN];

uint16_t elrs_channels[ELRS_CHAN_NUM];
rc_raw_ch rc_raw_channels;
elrs_status_t elrs_status = {0, 0, 0, 0};

/**
 * @brief  CRSF CRC-8 计算 (多项式 0xD5)
 * @param  type   帧类型字节
 * @param  data   负载数据指针
 * @param  len    数据长度
 * @return CRC 校验值
 */
uint8_t calc_crc(uint8_t type, const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    crc ^= type;
    for (uint8_t i = 0; i < 8; i++) 
        crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
    
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
    }
    return crc;
}

/** @brief 将 172-1811 映射到 -100~100 *//** author : lsl-sys*/
static int8_t remap_channel(uint16_t val)
{
    if (val < RC_MIN) return -100;
    if (val > RC_MAX) return 100;
    return (int8_t)((val - RC_MIN) * 200 / (RC_MAX - RC_MIN) - 100);
}

/**
 * @brief  解析 10 通道数据 (11bit/通道, 共 22 字节)
 * @param  payload 22字节负载数据
 */
static void parse_channels(const uint8_t *payload)
{
    uint64_t bits = 0;
    uint8_t bit_cnt = 0, byte_idx = 0;
    
    for (uint8_t i = 0; i < ELRS_CHAN_NUM; i++) {
        while (bit_cnt < 11) {
            bits |= ((uint64_t)payload[byte_idx++] << bit_cnt);
            bit_cnt += 8;
        }
        elrs_channels[i] = bits & 0x07FF;
        bits >>= 11;
        bit_cnt -= 11;
    }
    
    rc_raw_channels.ch1  = remap_channel(elrs_channels[0]);
    rc_raw_channels.ch2  = remap_channel(elrs_channels[1]);
    rc_raw_channels.ch3  = remap_channel(elrs_channels[2]);
    rc_raw_channels.ch4  = remap_channel(elrs_channels[3]);
    rc_raw_channels.ch5  = remap_channel(elrs_channels[4]);
    rc_raw_channels.ch6  = remap_channel(elrs_channels[5]);
    rc_raw_channels.ch7  = remap_channel(elrs_channels[6]);
    rc_raw_channels.ch8  = remap_channel(elrs_channels[7]);
    rc_raw_channels.ch9  = remap_channel(elrs_channels[8]);
    rc_raw_channels.ch10 = remap_channel(elrs_channels[9]);
}

/** @brief 进入失控保护状态 */
static void enter_failsafe(void)
{
#if ELRS_FAILSAFE_MODE == 1
    memset(&rc_raw_channels, 0, sizeof(rc_raw_channels));
#elif ELRS_FAILSAFE_MODE == 2
	  rc_raw_channels.ch1 = 0;
	  rc_raw_channels.ch2 = 0;
    rc_raw_channels.ch3 = -100;  // 油门最低
	  rc_raw_channels.ch4 = 0;     //其余保持
#endif
}

void crsf_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&ELRS_HUART, elrs_rx_buf, BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&ELRS_DMA_INSTANCE, DMA_IT_HT);
    memset(elrs_rx_buf, 0, BUF_SIZE);
}

/**
 * @brief  串口接收完成回调 (在中断中调用)
 */
void crsf_receive_data(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&ELRS_HUART, elrs_rx_buf, BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&ELRS_DMA_INSTANCE, DMA_IT_HT);
    elrs_flag_of_receive = 1;
}

/**
 * @brief  数据解析与状态检测 (在主循环或定时器中调用)
 * @note   自动处理帧查找、CRC 校验、超时判定和失控保护
 */
void crsf_analysis_data(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t frame_found = 0;
    
    if (elrs_flag_of_receive) {
        elrs_flag_of_receive = 0;
        
        // 帧头查找与 CRC 校验
        for (uint8_t i = 0; i <= BUF_SIZE - CRSF_FRAME_LEN; i++) {
            if (elrs_rx_buf[i] != CRSF_SYNC) continue;
            if (elrs_rx_buf[i+1] != (CRSF_PAYLOAD_LEN + 2)) continue;
            if (elrs_rx_buf[i+2] != CRSF_TYPE_RC) continue;
            
            uint8_t *p = &elrs_rx_buf[i];
            if (calc_crc(CRSF_TYPE_RC, &p[3], CRSF_PAYLOAD_LEN) != p[25])
                continue;
            
            // 校验通过，解析数据
            memcpy(elrs_payload, &p[3], CRSF_PAYLOAD_LEN);
            parse_channels(elrs_payload);
            
            elrs_status.frame_valid = 1;
            elrs_status.last_tick = now;
            frame_found = 1;
            
            if (!elrs_status.is_connected) {
                elrs_status.is_connected = 1;
                elrs_status.lost_count = 0;
            }
            
            memset(elrs_rx_buf, 0, BUF_SIZE);
            break;
        }
        
        if (!frame_found) {
            elrs_status.frame_valid = 0;
        }
    }
    
    // 超时判定
    if (now - elrs_status.last_tick > ELRS_TIMEOUT_MS) {
        if (elrs_status.is_connected) {
            // 首次超时，增加丢失计数
            elrs_status.lost_count++;
            
            // 只有连续超时达到容忍次数，才判定为失控
            if (elrs_status.lost_count >= ELRS_LOST_TOLERANCE) {
                elrs_status.is_connected = 0;
                enter_failsafe();
            }
        }
    }
    
    // 清理缓冲区
    if (!frame_found && elrs_flag_of_receive) {
        memset(elrs_rx_buf, 0, BUF_SIZE);
    }
}

#else

/* ================= SBUS 协议预留================= */

void sbus_init(void)
{
}

void sbus_receive_data(void)
{
}

void sbus_analysis_data(void)
{
}

#endif
