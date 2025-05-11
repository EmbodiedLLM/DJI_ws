#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "CRC8_CRC16.h"

// Protocol definitions
#define SOF                 0xA5  // Start of frame
#define ROS_VEL_CMD_ID      0x0201  // Command ID for ROS velocity command
#define ROS_DATA_LENGTH     6     // Data length for ROS velocity command (3 * uint16_t)

// External variables from CRC8_CRC16.c
extern const uint8_t CRC8_INIT;
extern uint16_t CRC16_INIT;

// Typedefs with special attention to byte order
#pragma pack(1)
typedef struct {
    uint16_t linear_x;    // Linear X velocity
    uint16_t linear_y;    // Linear Y velocity
    uint16_t angular_z;   // Angular Z velocity
} ros_ctrl_state_t;

typedef struct {
    uint8_t  sof;           // Start of frame, always 0xA5
    uint16_t data_length;   // Length of data in bytes
    uint8_t  seq;           // Sequence number
    uint8_t  crc8;          // CRC8 checksum of frame header
    uint16_t cmd_id;        // Command ID
    ros_ctrl_state_t data;  // Command data
    uint16_t crc16;         // CRC16 checksum of the entire frame
} __attribute__((packed)) ros_cmd_frame_t;
#pragma pack()

/**
 * @brief Print buffer contents in hexadecimal format
 * @param buffer Buffer to print
 * @param length Buffer length
 */
void print_hex_buffer(uint8_t *buffer, size_t length) {
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
}

/**
 * @brief Creates and prints a ROS velocity command with proper CRC checksums
 * @param linear_x Linear X velocity
 * @param linear_y Linear Y velocity
 * @param angular_z Angular Z velocity
 * @param seq Sequence number
 */
void create_ros_vel_cmd(uint16_t linear_x, uint16_t linear_y, uint16_t angular_z, uint8_t seq) {
    ros_cmd_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    
    // Initialize frame header
    frame.sof = SOF;
    frame.data_length = ROS_DATA_LENGTH;
    frame.seq = seq;
    
    // Calculate CRC8 for the frame header (SOF, data_length, seq)
    uint8_t *header = (uint8_t *)&frame;
    frame.crc8 = get_CRC8_check_sum(header, 4, CRC8_INIT);
    
    // Set command ID and data (ensuring proper byte order)
    frame.cmd_id = ROS_VEL_CMD_ID;
    frame.data.linear_x = linear_x;
    frame.data.linear_y = linear_y;
    frame.data.angular_z = angular_z;
    
    // Calculate CRC16 for the entire frame (excluding CRC16 itself)
    uint8_t *frame_bytes = (uint8_t *)&frame;
    frame.crc16 = get_CRC16_check_sum(frame_bytes, sizeof(frame) - 2, CRC16_INIT);
    
    // Print frame size
    printf("Frame size: %zu bytes\n", sizeof(frame));
    
    // Print the generated command in hex format
    printf("Generated command (hex): ");
    print_hex_buffer((uint8_t *)&frame, sizeof(frame));
    
    // Print the command in a more readable format with field descriptions
    printf("\nCommand breakdown (memory layout, may not match wire format):\n");
    printf("SOF:         %02X\n", frame.sof);
    printf("Data Length: %02X %02X\n", (uint8_t)frame.data_length, (uint8_t)(frame.data_length >> 8));
    printf("Sequence:    %02X\n", frame.seq);
    printf("Header CRC8: %02X\n", frame.crc8);
    printf("CMD ID:      %02X %02X\n", (uint8_t)frame.cmd_id, (uint8_t)(frame.cmd_id >> 8));
    printf("Data:        %02X %02X %02X %02X %02X %02X\n", 
           (uint8_t)frame.data.linear_x, (uint8_t)(frame.data.linear_x >> 8),
           (uint8_t)frame.data.linear_y, (uint8_t)(frame.data.linear_y >> 8),
           (uint8_t)frame.data.angular_z, (uint8_t)(frame.data.angular_z >> 8));
    printf("Frame CRC16: %02X %02X\n", (uint8_t)frame.crc16, (uint8_t)(frame.crc16 >> 8));
    
    // Create a buffer in the exact wire format described in the requirements
    printf("\nCorrect wire format (explicit byte order):\n");
    uint8_t wire_format[15];
    int idx = 0;
    
    // Header
    wire_format[idx++] = SOF;               // 0: SOF (0xA5)
    wire_format[idx++] = ROS_DATA_LENGTH;   // 1: Data length low byte (6)
    wire_format[idx++] = 0x00;              // 2: Data length high byte (0)
    wire_format[idx++] = seq;               // 3: Sequence number
    
    // Calculate header CRC8
    uint8_t header_crc8 = get_CRC8_check_sum(wire_format, 4, CRC8_INIT);
    wire_format[idx++] = header_crc8;       // 4: Header CRC8
    
    // Command ID (little-endian: 0x0201 -> 0x01 0x02)
    wire_format[idx++] = ROS_VEL_CMD_ID & 0xFF;         // 5: CMD ID low byte
    wire_format[idx++] = (ROS_VEL_CMD_ID >> 8) & 0xFF;  // 6: CMD ID high byte
    
    // Data (all little-endian)
    wire_format[idx++] = linear_x & 0xFF;         // 7: Linear X low byte
    wire_format[idx++] = (linear_x >> 8) & 0xFF;  // 8: Linear X high byte
    
    wire_format[idx++] = linear_y & 0xFF;         // 9: Linear Y low byte
    wire_format[idx++] = (linear_y >> 8) & 0xFF;  // 10: Linear Y high byte
    
    wire_format[idx++] = angular_z & 0xFF;        // 11: Angular Z low byte
    wire_format[idx++] = (angular_z >> 8) & 0xFF; // 12: Angular Z high byte
    
    // Calculate CRC16 for the entire packet
    uint16_t wire_crc16 = get_CRC16_check_sum(wire_format, idx, CRC16_INIT);
    
    // Add CRC16 (little-endian)
    wire_format[idx++] = wire_crc16 & 0xFF;        // 13: CRC16 low byte
    wire_format[idx++] = (wire_crc16 >> 8) & 0xFF; // 14: CRC16 high byte
    
    // Print the wire format
    printf("Wire format:  ");
    print_hex_buffer(wire_format, sizeof(wire_format));
    
    // Print with field explanation
    printf("\nWire format breakdown:\n");
    printf("SOF:         %02X\n", wire_format[0]);
    printf("Data Length: %02X %02X\n", wire_format[1], wire_format[2]);
    printf("Sequence:    %02X\n", wire_format[3]);
    printf("Header CRC8: %02X\n", wire_format[4]);
    printf("CMD ID:      %02X %02X\n", wire_format[5], wire_format[6]);
    printf("linear_x:    %02X %02X\n", wire_format[7], wire_format[8]);
    printf("linear_y:    %02X %02X\n", wire_format[9], wire_format[10]);
    printf("angular_z:   %02X %02X\n", wire_format[11], wire_format[12]);
    printf("Frame CRC16: %02X %02X\n", wire_format[13], wire_format[14]);
}

int main() {
    printf("Example 1 (from requirements):\n");
    printf("linear_x = 100 (0x0064), linear_y = 200 (0x00C8), angular_z = 50 (0x0032), seq = 0x42\n");
    create_ros_vel_cmd(100, 200, 50, 0x42);
    
    printf("\n------------------------------------------------\n\n");
    
    printf("Example 2 (zero values):\n");
    printf("linear_x = 0 (0x0000), linear_y = 0 (0x0000), angular_z = 0 (0x0000), seq = 0x00\n");
    create_ros_vel_cmd(0, 0, 0, 0x00);
    
    printf("\n------------------------------------------------\n\n");
    
    printf("Example 3 (maximum values):\n");
    printf("linear_x = 1000 (0x03E8), linear_y = 2000 (0x07D0), angular_z = 500 (0x01F4), seq = 0xFF\n");
    create_ros_vel_cmd(1000, 2000, 500, 0xFF);
    
    return 0;
} 