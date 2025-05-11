#ifndef DJI_PROTOCOL_HPP_
#define DJI_PROTOCOL_HPP_

#include <cstdint>

namespace dji_driver
{

#pragma pack(1)  // 确保结构体紧凑对齐，避免内存填充

/**
 * @brief 发送给驱动板的控制命令结构体
 * 
 * 该结构体定义了从ROS2节点发送到底盘控制板的控制命令格式。
 * 包含三个方向的速度指令，以及用于数据校验的头尾标识和校验和。
 */
struct ControlCommand {
    uint8_t header = 0xAA;  // 帧头标识，固定值0xAA
    float vx;               // X方向线速度，单位m/s，前进为正
    float vy;               // Y方向线速度，单位m/s，左移为正
    float wz;               // Z轴角速度，单位rad/s，逆时针为正
    uint8_t checksum;       // 校验和，用于验证数据完整性
    uint8_t footer = 0x55;  // 帧尾标识，固定值0x55
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

/**
 * @brief 计算校验和
 * 
 * @param data 需要计算校验和的数据缓冲区指针
 * @param length 数据长度
 * @return uint8_t 计算得到的校验和
 * 
 * 采用简单的加和校验，将所有字节相加得到的结果作为校验和。
 * 注意：实际应用中可能需要更复杂的校验算法，如CRC16等。
 */
inline uint8_t calculate_checksum(const uint8_t* data, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += data[i];
    }
    return sum;
}

} // namespace dji_driver

#endif // DJI_PROTOCOL_HPP_ 