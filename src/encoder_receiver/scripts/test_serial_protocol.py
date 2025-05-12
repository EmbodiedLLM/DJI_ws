#!/usr/bin/env python3
import serial
import struct
import time
import argparse
import binascii

# DJI Protocol definitions
HEADER_SOF = 0xA5
CHASSIS_STATUS_CMD_ID = 0x0301

class DJIProtocolParser:
    def __init__(self, port='/dev/ttyACM2', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.buffer = bytearray()
        self.frame_max_size = 128
        
        # State machine states
        self.STEP_SOF = 0
        self.STEP_LENGTH_LOW = 1
        self.STEP_LENGTH_HIGH = 2
        self.STEP_SEQ = 3
        self.STEP_CRC8 = 4
        self.STEP_DATA = 5
        
        self.state = self.STEP_SOF
        self.data_length = 0
        self.frame_length = 0
        
    def read_and_parse(self):
        if self.ser.in_waiting:
            data = self.ser.read(self.ser.in_waiting)
            for byte in data:
                if self.process_byte(byte):
                    # Frame processed
                    pass
    
    def process_byte(self, byte):
        # Add byte to buffer
        self.buffer.append(byte)
        
        # Parse according to state machine
        if self.state == self.STEP_SOF:
            if byte == HEADER_SOF:
                self.buffer = bytearray([byte])
                self.state = self.STEP_LENGTH_LOW
            else:
                self.buffer = bytearray()
            return False
                
        elif self.state == self.STEP_LENGTH_LOW:
            self.data_length = byte
            self.state = self.STEP_LENGTH_HIGH
            return False
                
        elif self.state == self.STEP_LENGTH_HIGH:
            self.data_length |= (byte << 8)
            self.state = self.STEP_SEQ
            return False
                
        elif self.state == self.STEP_SEQ:
            self.state = self.STEP_CRC8
            return False
                
        elif self.state == self.STEP_CRC8:
            # For simplicity, we're not verifying CRC8 here
            self.state = self.STEP_DATA
            # Header + CMD + data + CRC16
            self.frame_length = 5 + 2 + self.data_length + 2
            
            # Check if length is valid
            if self.data_length > self.frame_max_size - 5 - 2 - 2:
                print(f"Invalid frame length: {self.data_length}")
                self.buffer = bytearray()
                self.state = self.STEP_SOF
            return False
                
        elif self.state == self.STEP_DATA:
            # Check if we have the complete frame
            if len(self.buffer) >= self.frame_length:
                # Process the complete frame
                self.process_frame(self.buffer[:self.frame_length])
                
                # Reset state machine
                self.buffer = bytearray()
                self.state = self.STEP_SOF
                return True
            return False
        
        return False
    
    def process_frame(self, frame):
        # Need at least header + cmd_id
        if len(frame) < 7:  # 5 header + 2 cmd_id
            return
        
        # Extract cmd_id (little endian)
        cmd_id = frame[5] | (frame[6] << 8)
        
        print(f"Frame received: {binascii.hexlify(frame).decode()}")
        print(f"Command ID: 0x{cmd_id:04X}")
        
        # Process based on cmd_id
        if cmd_id == CHASSIS_STATUS_CMD_ID:
            # Process encoder data
            if len(frame) >= 9 + 8:  # header + cmd_id + at least 4 encoders (2 bytes each)
                encoder_data = struct.unpack('<HHHH', frame[7:15])
                print(f"Encoder counts: {encoder_data}")
        
def main():
    parser = argparse.ArgumentParser(description='Test DJI Serial Protocol')
    parser.add_argument('--port', type=str, default='/dev/ttyACM2', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()
    
    print(f"Opening serial port {args.port} at {args.baud} baud")
    dji_parser = DJIProtocolParser(args.port, args.baud)
    
    try:
        while True:
            dji_parser.read_and_parse()
            time.sleep(0.01)  # Small delay to prevent CPU hogging
    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if hasattr(dji_parser, 'ser') and dji_parser.ser.is_open:
            dji_parser.ser.close()
            print("Serial port closed")

if __name__ == '__main__':
    main() 