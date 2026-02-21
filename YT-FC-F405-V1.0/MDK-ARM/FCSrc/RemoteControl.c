#include "RemoteControl.h"
#include "ELRS.h"  // 包含ELRS驱动，用于获取连接状态

/**
 * @brief 外部变量引用
 */
extern rc_raw_ch rc_raw_channels;  /* ELRS解析后的原始遥控器数据（范围-100~100） */

/**
 * @brief 全局变量定义
 */
rc_channels filtered_rc;           /* 经过滤波处理后的最终通道值 */

/**
 * @brief 静态变量定义（内部状态维护）
 */
static rc_channels last_valid_rc;   /* 上一次有效的通道值，用于突变检测和保持 */
static uint8_t rc_initialized = 0;  /* 系统初始化标志：0-首次运行，1-已初始化 */
static uint8_t init_counter = 0;    /* 初始化计数器：前20帧用于稳定初始值 */
static uint8_t abnormal_frame_count = 0; /* 连续异常帧计数，用于整帧错误判定 */

/**
 * @brief 新增：ELRS连接状态跟踪
 * @note 用于检测"刚刚恢复连接"的瞬间，避免从failsafe值(-100)跳变到正常值被判定为突变
 */
static uint8_t last_connected = 1;  /* 上一帧的连接状态：1-正常，0-失控 */

/**
 * @brief 宏定义
 */
#define ABNORMAL_FRAME_THRESHOLD 5  /* 整帧异常阈值：连续5帧异常则判定为数据错误 */

/**
 * @brief  检查按钮通道（ch8）的有效性
 * @param  current 当前值
 * @param  last 上一次有效值（保留参数，当前实现完全放行）
 * @param  channel 通道号（保留参数，用于扩展）
 * @return 1-有效，0-无效
 * @note   按钮通道通常只有按下/松开两种状态，此处完全信任硬件
 */
static uint8_t is_valid_button(int8_t current, int8_t last, uint8_t channel) {
    (void)last; (void)channel;  // 消除未使用参数警告
    return 1;  // 完全放行，不做限制
}

/**
 * @brief  检查自恢复按钮通道（ch9）的有效性
 * @param  current 当前值
 * @param  last 上一次有效值
 * @return 1-有效，0-无效
 * @note   自恢复按钮特性：松手自动回弹到-100，按下时为100
 *         必须允许从100自动回到-100的过程
 */
static uint8_t is_valid_se_switch(int8_t current, int8_t last) {
    // 松开状态：-100附近（允许±10误差）
    if (current >= -100 && current <= -90) return 1;
    
    // 按下状态：100附近（允许±10误差）
    if (current >= 90 && current <= 100) return 1;
    
    // 保持状态：与上次相同（消抖）
    if (current == last) return 1;
    
    return 0;  // 其他中间值视为无效
}

/**
 * @brief  检查三状态开关通道（ch5、ch6、ch7）的有效性
 * @param  current 当前值
 * @param  last 上一次有效值（保留参数，当前只检查合法位置）
 * @return 1-有效，0-无效
 * @note   三状态开关只有三个合法位置：-100（下）、0（中）、100（上）
 */
static uint8_t is_valid_three_state(int8_t current, int8_t last) {
    (void)last;
    // 三个状态位置，各允许±10的抖动范围
    if ((current >= -100 && current <= -90) ||  // 下档
        (current >= -5 && current <= 5) ||       // 中档
        (current >= 90 && current <= 100)) {     // 上档
        return 1;
    }
    return 0;
}

/**
 * @brief  检查滚轮通道（ch10）的有效性
 * @param  current 当前值
 * @param  last 上一次有效值
 * @return 1-有效，0-无效
 * @note   滚轮常见异常值：-35、-65等固定尖峰
 *         正常范围应在-100~100之间
 */
static uint8_t is_valid_roller(int8_t current, int8_t last) {
    (void)last;
    // 排除已知的异常尖峰值
    if (current == -35 || current == -65) return 0;
    
    // 排除超界值
    if (current < -100 || current > 100) return 0;
    
    return 1;
}

/**
 * @brief  整帧数据一致性检查
 * @param  raw 指向原始数据的指针
 * @return 1-整帧可信，0-可能存在传输错误
 * @note   通过检测多个通道同时跳变来识别整帧错位或串扰
 *         初始化阶段（前20帧）跳过检查，避免误判初始值
 */
static uint8_t is_valid_frame(rc_raw_ch *raw) {
    // 初始化期：前20帧不检查，积累历史数据
    if (init_counter < 20) {
        return 1;
    }
    
    uint8_t abnormal_count = 0;
    
    // 摇杆通道（ch1-ch4）：检测小幅跳变（>10视为异常）
    // 注意：这里是与last_valid_rc比较，不是与当前raw比较
    if (abs(raw->ch1 - last_valid_rc.RX) > 10) abnormal_count++;
    if (abs(raw->ch2 - last_valid_rc.RY) > 10) abnormal_count++;
    if (abs(raw->ch3 - last_valid_rc.LY) > 10) abnormal_count++;
    if (abs(raw->ch4 - last_valid_rc.LX) > 10) abnormal_count++;
    
    // 开关通道（ch5-ch9）：检测大幅跳变（>50视为异常）
    if (abs(raw->ch5 - last_valid_rc.SA) > 50) abnormal_count++;
    if (abs(raw->ch6 - last_valid_rc.SB) > 50) abnormal_count++;
    if (abs(raw->ch7 - last_valid_rc.SC) > 50) abnormal_count++;
    if (abs(raw->ch8 - last_valid_rc.SD) > 50) abnormal_count++;
    if (abs(raw->ch9 - last_valid_rc.SE) > 50) abnormal_count++;
    
    // 如果3个及以上通道同时突变，判定为整帧错误（如DMA错位）
    if (abnormal_count >= 3) {
        return 0;
    }
    
    return 1;
}

void rc_init(void)
{
	elrs_init();
}

/**
 * @brief  遥控器数据滤波处理主函数
 * @param  无
 * @return 无
 * @note   调用频率：建议1ms一次（与ELRS解析频率匹配）
 * @warning 此函数会修改全局变量 filtered_rc
 * 
 * 处理流程：
 * 1. 检测ELRS连接状态（失控/恢复/正常）
 * 2. 失控状态：强制采用failsafe值，重置历史记录
 * 3. 恢复瞬间：重置历史记录，消除从failsafe到正常值的突变
 * 4. 正常状态：执行常规的尖峰滤波和有效性检查
 */
void rc_filter_process(void) {
    // 获取ELRS当前连接状态（0=失控/超时，1=正常接收）
    // elrs_is_connected() 在ELRS.h中定义为内联函数
    uint8_t now_connected = elrs_is_connected();
    
    /* ==================== 阶段一：失控状态处理 ====================
     * 当遥控器丢失信号时，ELRS会调用enter_failsafe()设置特定值（如油门-100）
     * 此时必须强制采用这些值，且不能视为"突变"
     * 同时重置所有历史记录，为恢复做准备
     */
    if (!now_connected) {
        // 强制同步：直接使用ELRS设置的failsafe值（不经过突变检测）
        filtered_rc.RX = rc_raw_channels.ch1;
        filtered_rc.RY = rc_raw_channels.ch2;
        filtered_rc.LY = rc_raw_channels.ch3;   // 通常为-100（油门最低）
        filtered_rc.LX = rc_raw_channels.ch4;
        filtered_rc.SA = rc_raw_channels.ch5;
        filtered_rc.SB = rc_raw_channels.ch6;
        filtered_rc.SC = rc_raw_channels.ch7;
        filtered_rc.SD = rc_raw_channels.ch8;
        filtered_rc.SE = rc_raw_channels.ch9;
        filtered_rc.SL = rc_raw_channels.ch10;
        
        // 关键：更新历史值，避免恢复时检测到"从旧值跳变到-100"的突变
        memcpy(&last_valid_rc, &filtered_rc, sizeof(rc_channels));
        
        // 重置初始化状态：让系统准备好重新初始化
        rc_initialized = 0;
        init_counter = 0;
        abnormal_frame_count = 0;
        
        // 记录状态，供下次判断"是否刚刚恢复"
        last_connected = 0;
        
        return;  // 失控状态下跳过所有滤波逻辑，直接返回
    }
    /** author : lsl-sys*/
    /* ==================== 阶段二：恢复瞬间处理 ====================
     * 当信号从丢失恢复到正常时，raw值会从failsafe值（如-100）跳变到正常值（如0）
     * 如果不处理，is_valid_frame()会判定这是"从-100到0的突变"而拒绝更新
     * 因此必须在此重置历史值，让系统认为这是"初始值"而非"突变"
     */
    if (now_connected && !last_connected) {
        // 将历史值设置为当前正常值，消除差值
        last_valid_rc.RX = rc_raw_channels.ch1;
        last_valid_rc.RY = rc_raw_channels.ch2;
        last_valid_rc.LY = rc_raw_channels.ch3;
        last_valid_rc.LX = rc_raw_channels.ch4;
        last_valid_rc.SA = rc_raw_channels.ch5;
        last_valid_rc.SB = rc_raw_channels.ch6;
        last_valid_rc.SC = rc_raw_channels.ch7;
        last_valid_rc.SD = rc_raw_channels.ch8;
        last_valid_rc.SE = rc_raw_channels.ch9;
        last_valid_rc.SL = rc_raw_channels.ch10;
        
        // 第一帧直接输出，不进行突变检测（相当于重新初始化）
        memcpy(&filtered_rc, &last_valid_rc, sizeof(rc_channels));
        
        // 设置初始化标志，但跳过缓慢的初始化期（直接进入稳定滤波）
        rc_initialized = 1;
        init_counter = 20;  // 直接设为20，跳过初始化保护期
        
        // 重置异常计数
        abnormal_frame_count = 0;
        
        last_connected = 1;
        return;  // 恢复的第一帧直接采用，不进行后续滤波
    }
    
    // 更新连接状态记录（正常状态下每次都要更新）
    last_connected = now_connected;
    
    /* ==================== 阶段三：正常滤波处理 ====================
     * 以下是你原有的正常状态下的滤波逻辑，保持不变
     */
    
    // 初始化计数器递增（前20帧用于稳定初始值）
    if (init_counter < 20) {
        init_counter++;
    }
    
    // 首次初始化：直接采用当前值，不滤波
    if (!rc_initialized) {
        filtered_rc.RX = rc_raw_channels.ch1;
        filtered_rc.RY = rc_raw_channels.ch2;
        filtered_rc.LY = rc_raw_channels.ch3;
        filtered_rc.LX = rc_raw_channels.ch4;
        filtered_rc.SA = rc_raw_channels.ch5;
        filtered_rc.SB = rc_raw_channels.ch6;
        filtered_rc.SC = rc_raw_channels.ch7;
        filtered_rc.SD = rc_raw_channels.ch8;
        filtered_rc.SE = rc_raw_channels.ch9;
        filtered_rc.SL = rc_raw_channels.ch10;
        
        // 保存为历史值
        memcpy(&last_valid_rc, &filtered_rc, sizeof(rc_channels));
        
        rc_initialized = 1;
        return;
    }
    
    // 缓存非摇杆通道的原始值（避免在多个检查中重复访问结构体）
    int8_t raw_ch5 = rc_raw_channels.ch5;
    int8_t raw_ch6 = rc_raw_channels.ch6;
    int8_t raw_ch7 = rc_raw_channels.ch7;
    int8_t raw_ch8 = rc_raw_channels.ch8;
    int8_t raw_ch9 = rc_raw_channels.ch9;
    int8_t raw_ch10 = rc_raw_channels.ch10;
    
    // 整帧一致性检查
    if (is_valid_frame(&rc_raw_channels)) {
        // 整帧正常：重置异常计数
        abnormal_frame_count = 0;
        
        // 摇杆通道（ch1-ch4）：采用当前值，但过滤向-100的突变尖峰
        // 如果上次值>-50（中位附近），这次突然变成-100（最小值），则视为尖峰
        filtered_rc.RX = rc_raw_channels.ch1;
        filtered_rc.RY = rc_raw_channels.ch2;
        filtered_rc.LY = rc_raw_channels.ch3;
        filtered_rc.LX = rc_raw_channels.ch4;
        
        // 尖峰过滤：只允许保持或小幅变化，不允许突跳到-100
        if (last_valid_rc.RX > -50 && filtered_rc.RX == -100) {
            filtered_rc.RX = last_valid_rc.RX;
        }
        if (last_valid_rc.RY > -50 && filtered_rc.RY == -100) {
            filtered_rc.RY = last_valid_rc.RY;
        }
        if (last_valid_rc.LY > -50 && filtered_rc.LY == -100) {
            filtered_rc.LY = last_valid_rc.LY;
        }
        if (last_valid_rc.LX > -50 && filtered_rc.LX == -100) {
            filtered_rc.LX = last_valid_rc.LX;
        }
        
        // 开关通道：通过各自的合法性检查
        if (is_valid_three_state(raw_ch5, filtered_rc.SA)) {
            filtered_rc.SA = raw_ch5;
        }
        if (is_valid_three_state(raw_ch6, filtered_rc.SB)) {
            filtered_rc.SB = raw_ch6;
        }
        if (is_valid_three_state(raw_ch7, filtered_rc.SC)) {
            filtered_rc.SC = raw_ch7;
        }
        if (is_valid_button(raw_ch8, filtered_rc.SD, 8)) {
            filtered_rc.SD = raw_ch8;
        }
        if (is_valid_se_switch(raw_ch9, filtered_rc.SE)) {
            filtered_rc.SE = raw_ch9;
        }
        if (is_valid_roller(raw_ch10, filtered_rc.SL)) {
            filtered_rc.SL = raw_ch10;
        }
        
    } else {
        // 整帧异常：增加计数
        abnormal_frame_count++;
        
        // 摇杆通道：保持上次值（不更新）
        // 注意：filtered_rc的摇杆值保持原样
        
        // 非摇杆通道：也保持上次值，但自恢复按钮特殊处理
        if (is_valid_se_switch(raw_ch9, filtered_rc.SE)) {
            filtered_rc.SE = raw_ch9;  // 即使整帧异常，也允许按钮回弹
        }
    }
    
    // 保存当前滤波值作为下次的历史参考
    memcpy(&last_valid_rc, &filtered_rc, sizeof(rc_channels));
}
