#ifndef DJI_PROTOCOL_HPP_
#define DJI_PROTOCOL_HPP_

#include <cstdint>

namespace dji_driver
{

#pragma pack(1)  // 确保结构体紧凑对齐

// 发送给驱动板的控制命令结构体
struct ControlCommand {
    uint8_t header = 0xAA;  // 帧头
    float vx;               // X方向线速度
    float vy;               // Y方向线速度
    float wz;               // Z轴角速度
    uint8_t checksum;       // 校验和
    uint8_t footer = 0x55;  // 帧尾
};

// 从驱动板接收的编码器数据结构体
struct EncoderData {
    uint8_t header = 0xBB;      // 帧头
    int32_t fl_encoder;         // 左前轮
    int32_t fr_encoder;         // 右前轮
    int32_t rl_encoder;         // 左后轮
    int32_t rr_encoder;         // 右后轮
    uint8_t checksum;           // 校验和
    uint8_t footer = 0x55;      // 帧尾
};

#pragma pack()

// 计算校验和
inline uint8_t calculate_checksum(const uint8_t* data, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += data[i];
    }
    return sum;
}

} // namespace dji_driver

#endif // DJI_PROTOCOL_HPP_ 