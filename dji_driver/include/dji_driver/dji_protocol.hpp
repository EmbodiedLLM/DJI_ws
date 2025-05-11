#ifndef DJI_PROTOCOL_HPP_
#define DJI_PROTOCOL_HPP_

#include <cstdint>

namespace dji_driver
{

// 协议常量定义
constexpr uint8_t SOF = 0xA5;         // 帧起始标识
constexpr uint16_t ROS_VEL_CMD_ID = 0x0201;  // 速度控制命令ID
constexpr uint8_t ROS_DATA_LENGTH = 6;       // 速度控制数据长度 (3个16位速度值)

// DJI底盘速度范围常量
constexpr int16_t DJI_SPEED_MIN = 364;   // 最小速度值
constexpr int16_t DJI_SPEED_MAX = 1684;  // 最大速度值
constexpr int16_t DJI_SPEED_MIDDLE = 1024; // 中间值（停止）

#pragma pack(1)  // 确保结构体紧凑对齐，避免内存填充

/**
 * @brief 发送给驱动板的控制命令结构体
 * 
 * 该结构体定义了从ROS2节点发送到底盘控制板的控制命令格式。
 * 包含三个方向的速度指令，以及用于数据校验的头尾标识和校验和。
 */
struct ControlCommand {
    uint8_t sof = SOF;        // 帧头标识，固定值0xA5
    uint16_t data_length = ROS_DATA_LENGTH;  // 数据长度，固定为6字节
    uint8_t seq = 0;          // 序列号，每次发送递增
    uint8_t crc8;             // 帧头校验和
    uint16_t cmd_id = ROS_VEL_CMD_ID;  // 命令ID，速度控制为0x0201
    int16_t linear_x;         // X方向线速度，单位mm/s
    int16_t linear_y;         // Y方向线速度，单位mm/s
    int16_t angular_z;        // Z轴角速度，单位0.1°/s
    uint16_t crc16;           // 全帧校验和
};

/**
 * @brief 从驱动板接收的编码器数据结构体
 * 
 * 该结构体定义了从底盘控制板接收的编码器数据格式。
 * 包含四个轮子的编码器计数值，用于计算里程计。
 */
struct EncoderData {
    uint8_t header = 0xBB;      // 帧头标识，固定值0xBB
    int32_t fl_encoder;         // 左前轮编码器计数值
    int32_t fr_encoder;         // 右前轮编码器计数值
    int32_t rl_encoder;         // 左后轮编码器计数值
    int32_t rr_encoder;         // 右后轮编码器计数值
    uint8_t checksum;           // 校验和，用于验证数据完整性
    uint8_t footer = 0x55;      // 帧尾标识，固定值0x55
};

#pragma pack()

// CRC8表及初始值
extern const uint8_t CRC8_INIT;
extern const uint8_t CRC8_TABLE[256];

// CRC16表及初始值
extern uint16_t CRC16_INIT;
extern const uint16_t CRC16_TABLE[256];

/**
 * @brief 计算CRC8校验和
 * 
 * @param data 需要计算校验和的数据缓冲区指针
 * @param length 数据长度
 * @param init_crc 初始CRC8值
 * @return uint8_t 计算得到的校验和
 */
uint8_t calculate_crc8(const uint8_t* data, uint32_t length, uint8_t init_crc);

/**
 * @brief 计算CRC16校验和
 * 
 * @param data 需要计算校验和的数据缓冲区指针
 * @param length 数据长度
 * @param init_crc 初始CRC16值
 * @return uint16_t 计算得到的校验和
 */
uint16_t calculate_crc16(const uint8_t* data, uint32_t length, uint16_t init_crc);

/**
 * @brief 填充控制命令的校验和
 * 
 * 计算并填充CRC8和CRC16校验和到控制命令结构体中
 * @param cmd 要填充校验和的控制命令结构体
 */
void fill_checksums(ControlCommand& cmd);

/**
 * @brief 验证编码器数据的校验和
 * 
 * 计算并验证接收到的编码器数据校验和是否正确
 * @param data 收到的编码器数据结构体
 * @return bool 校验和是否正确
 */
bool verify_encoder_checksum(const EncoderData& data);

/**
 * @brief 计算编码器数据的校验和
 * 
 * 计算编码器数据的校验和，简单的字节累加
 * @param data 编码器数据结构体
 * @return uint8_t 计算得到的校验和
 */
uint8_t calculate_encoder_checksum(const EncoderData& data);

} // namespace dji_driver

#endif // DJI_PROTOCOL_HPP_ 