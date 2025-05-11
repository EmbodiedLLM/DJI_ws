#include "dji_driver/dji_protocol.hpp"
#include <cstddef>  // 添加std::size_t的定义

// External variables from CRC8_CRC16.c - defined in global namespace
extern "C" {
    extern const uint8_t CRC8_INIT;
    extern uint16_t CRC16_INIT;
}

namespace dji_driver
{

/**
 * @brief 填充控制命令的校验和
 * 
 * 计算并填充CRC8和CRC16校验和到控制命令结构体中
 * @param cmd 要填充校验和的控制命令结构体
 */
void fill_checksums(ControlCommand& cmd)
{
    // 计算并填充CRC8（帧头校验，前4字节）
    cmd.crc8 = get_CRC8_check_sum(reinterpret_cast<unsigned char*>(&cmd), 4, CRC8_INIT);
    
    // 计算并填充CRC16（整个数据包校验，不包括CRC16本身）
    cmd.crc16 = get_CRC16_check_sum(reinterpret_cast<uint8_t*>(&cmd), 
                                 sizeof(ControlCommand) - 2, CRC16_INIT);
}

/**
 * @brief 计算编码器数据的校验和
 * 
 * 计算编码器数据的校验和，简单的字节累加
 * @param data 编码器数据结构体
 * @return uint8_t 计算得到的校验和
 */
uint8_t calculate_encoder_checksum(const EncoderData& data)
{
    uint8_t checksum = 0;
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&data);
    
    // 计算所有字段的和，不包括校验和字段和帧尾
    // 1字节header + 16字节(4x 4字节的编码器数据)
    for (std::size_t i = 0; i < sizeof(EncoderData) - 2; ++i)
    {
        checksum += ptr[i];
    }
    
    return checksum;
}

/**
 * @brief 验证编码器数据的校验和
 * 
 * 计算并验证接收到的编码器数据校验和是否正确
 * @param data 收到的编码器数据结构体
 * @return bool 校验和是否正确
 */
bool verify_encoder_checksum(const EncoderData& data)
{
    uint8_t calculated_checksum = calculate_encoder_checksum(data);
    return calculated_checksum == data.checksum;
}

} // namespace dji_driver