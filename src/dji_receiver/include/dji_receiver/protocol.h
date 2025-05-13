#ifndef ROS_CMD_PROTOCOL_H
#define ROS_CMD_PROTOCOL_H

#include <stdint.h>

#define HEADER_SOF 0xA5
#define ROS_PROTOCOL_FRAME_MAX_SIZE         128

#define ROS_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define ROS_PROTOCOL_CMD_SIZE               2
#define ROS_PROTOCOL_CRC16_SIZE             2
#define ROS_HEADER_CRC_LEN                  (ROS_PROTOCOL_HEADER_SIZE + ROS_PROTOCOL_CRC16_SIZE)
#define ROS_HEADER_CRC_CMDID_LEN            (ROS_PROTOCOL_HEADER_SIZE + ROS_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define ROS_HEADER_CMDID_LEN                (ROS_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
    // GAME_STATE_CMD_ID                 = 0x0001,
    // GAME_RESULT_CMD_ID                = 0x0002,
    // GAME_ROBOT_HP_CMD_ID              = 0x0003,
    // FIELD_EVENTS_CMD_ID               = 0x0101,
    // SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,
    // SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,
    // REFEREE_WARNING_CMD_ID            = 0x0104,
    ROS_VEL_CMD_ID                = 0x0201,
    CHASSIS_STATUS_CMD_ID         = 0x0301,
    // POWER_HEAT_DATA_CMD_ID            = 0x0202,
    // ROBOT_POS_CMD_ID                  = 0x0203,
    // BUFF_MUSK_CMD_ID                  = 0x0204,
    // AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,
    // ROBOT_HURT_CMD_ID                 = 0x0206,
    // SHOOT_DATA_CMD_ID                 = 0x0207,
    // BULLET_REMAINING_CMD_ID           = 0x0208,
    // STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,
    // IDCustomData,
}referee_cmd_id_t;
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[ROS_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

typedef union {
    float f;
    uint8_t u8[4];
} float_uint8_union_t;

typedef struct __attribute__((packed)) {
    int16_t motor_speed[4];       // 四个电机的速度
    
    float_uint8_union_t vx;       // 底盘x方向速度
    float_uint8_union_t vy;       // 底盘y方向速度
    float_uint8_union_t wz;       // 底盘旋转速度
    
    float_uint8_union_t chassis_yaw;    // 底盘航向角
    float_uint8_union_t chassis_pitch;  // 底盘俯仰角
    float_uint8_union_t chassis_roll;   // 底盘横滚角
} chassis_upload_data_t;


struct chassis_move_t {
    float vx;
    float vy;
    float vz;
    float wx;
    float wy;
    float wz;
    float yaw;
    float pitch;
    float roll;
};

#pragma pack(pop)

#endif //ROS_CMD_PROTOCOL_H
