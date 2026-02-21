#ifndef __ELRS_REG_H
#define __ELRS_REG_H
/**
 * @brief CRSF协议地址枚举
 */
typedef enum
{
    CRSF_ADDRESS_BROADCAST = 0x00,          /*!< 广播地址 */
    CRSF_ADDRESS_USB = 0x10,                /*!< USB设备地址 */
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,   /*!< TBS Core PNP Pro设备地址 */
    CRSF_ADDRESS_RESERVED1 = 0x8A,          /*!< 保留地址1 */
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,     /*!< 电流传感器地址 */
    CRSF_ADDRESS_GPS = 0xC2,                /*!< GPS模块地址 */
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,       /*!< TBS黑盒地址 */
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,  /*!< 飞控地址 */
    CRSF_ADDRESS_RESERVED2 = 0xCA,          /*!< 保留地址2 */
    CRSF_ADDRESS_RACE_TAG = 0xCC,           /*!< 竞赛标签地址 */
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,  /*!< 无线电发射机地址 */
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,      /*!< CRSF接收机地址 */
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,   /*!< CRSF发射机地址 */
} crsf_addr_e;

/**
 * @brief CRSF协议帧类型枚举
 */
typedef enum
{
    CRSF_FRAMETYPE_GPS = 0x02,                    /*!< GPS数据帧 */
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,         /*!< 电池传感器数据帧 */
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,        /*!< 链路统计信息帧 */
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,            /*!< OpenTX同步帧 */
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,               /*!< 无线电ID帧 */
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,     /*!< 打包的RC通道数据帧 */
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,               /*!< 姿态数据帧 */
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,            /*!< 飞行模式帧 */
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,            /*!< 设备PING帧 */
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,            /*!< 设备信息帧 */
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B, /*!< 参数设置条目帧 */
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,         /*!< 参数读取帧 */
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,        /*!< 参数写入帧 */
    CRSF_FRAMETYPE_COMMAND = 0x32,                /*!< 命令帧 */
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,                /*!< MSP请求帧 */
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,               /*!< MSP响应帧 */
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C,              /*!< MSP写入帧 */
} crsf_frame_type_e;

#endif
