#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <serial/serial.h>

#include <cstring>
#include <memory>
#include <cmath>
#include <string>
#include <vector>

// DJI Protocol definitions
#define HEADER_SOF 0xA5
#define ROS_PROTOCOL_FRAME_MAX_SIZE 128
#define ROS_PROTOCOL_HEADER_SIZE 5
#define ROS_PROTOCOL_CMD_SIZE 2
#define ROS_PROTOCOL_CRC16_SIZE 2
#define CHASSIS_STATUS_CMD_ID 0x0301

// Packed structures for protocol parsing
#pragma pack(push, 1)
typedef struct {
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_t;

typedef struct {
    uint16_t encoder_counts[4]; // 4 motor encoder data
} chassis_encoder_data_t;
#pragma pack(pop)

class DjiEncoderReceiver : public rclcpp::Node {
public:
    DjiEncoderReceiver() : Node("dji_encoder_receiver") {
        // Declare parameters
        declare_parameter("serial_port", "/dev/dji_serial");
        declare_parameter("baud_rate", 115200);
        declare_parameter("wheel_radius", 0.076);  // meters
        declare_parameter("wheel_base", 0.4);      // meters
        declare_parameter("encoder_counts_per_rev", 8192.0);  // Adjust based on motor specs
        declare_parameter("publish_rate", 50.0);   // Hz
        declare_parameter("frame_id", "odom");
        declare_parameter("child_frame_id", "base_link");

        // Get parameters
        serial_port_ = get_parameter("serial_port").as_string();
        baud_rate_ = get_parameter("baud_rate").as_int();
        wheel_radius_ = get_parameter("wheel_radius").as_double();
        wheel_base_ = get_parameter("wheel_base").as_double();
        encoder_counts_per_rev_ = get_parameter("encoder_counts_per_rev").as_double();
        publish_rate_ = get_parameter("publish_rate").as_double();
        frame_id_ = get_parameter("frame_id").as_string();
        child_frame_id_ = get_parameter("child_frame_id").as_string();

        // Initialize publishers
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // Initialize timer for publishing
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&DjiEncoderReceiver::timer_callback, this));
        
        // Initialize serial port
        try {
            serial_ = std::make_unique<serial::Serial>(
                serial_port_,
                baud_rate_,
                serial::Timeout::simpleTimeout(1000)
            );
            
            if (serial_->isOpen()) {
                RCLCPP_INFO(get_logger(), "Serial port %s opened successfully", serial_port_.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", serial_port_.c_str());
                return;
            }
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(get_logger(), "Exception opening serial port: %s", e.what());
            return;
        }
        
        // Initialize odometry state
        x_ = y_ = theta_ = 0.0;
        vx_ = vy_ = vtheta_ = 0.0;
        
        // Initialize previous encoder values
        for (int i = 0; i < 4; i++) {
            prev_encoder_counts_[i] = 0;
            encoder_initialized_[i] = false;
        }
        
        RCLCPP_INFO(get_logger(), "DJI Encoder Receiver initialized");
    }
    
    ~DjiEncoderReceiver() {
        if (serial_ && serial_->isOpen()) {
            serial_->close();
            RCLCPP_INFO(get_logger(), "Serial port closed");
        }
    }
    
private:
    void timer_callback() {
        if (!serial_ || !serial_->isOpen()) {
            RCLCPP_ERROR(get_logger(), "Serial port not open");
            return;
        }
        
        process_serial_data();
        // publish_odometry();
    }
    
    void process_serial_data() {
        size_t bytes_available = serial_->available();
        if (bytes_available < ROS_PROTOCOL_HEADER_SIZE) {
            return;
        }
        
        // Read all available data
        std::vector<uint8_t> buffer(bytes_available);
        serial_->read(buffer.data(), bytes_available);
        
        // Process all received bytes
        for (size_t i = 0; i < bytes_available; i++) {
            uint8_t byte = buffer[i];
            bool frame_processed = process_byte(byte);
            
            if (frame_processed) {
                RCLCPP_DEBUG(get_logger(), "Frame processed successfully");
            }
        }
        // log out the buffer
        RCLCPP_INFO(get_logger(), "Buffer: %s", buffer.data());
    }
    
    bool process_byte(uint8_t byte) {
        static std::vector<uint8_t> frame_buffer(ROS_PROTOCOL_FRAME_MAX_SIZE);
        static size_t buffer_index = 0;
        static enum {
            STEP_SOF,
            STEP_LENGTH_LOW,
            STEP_LENGTH_HIGH,
            STEP_SEQ,
            STEP_CRC8,
            STEP_DATA
        } parse_state = STEP_SOF;
        
        static uint16_t frame_length = 0;
        static uint16_t data_length = 0;
        
        bool frame_processed = false;
        
        // Add byte to buffer
        frame_buffer[buffer_index++] = byte;
        
        // Parse according to state machine
        switch (parse_state) {
            case STEP_SOF:
                if (byte == HEADER_SOF) {
                    buffer_index = 1;
                    parse_state = STEP_LENGTH_LOW;
                } else {
                    buffer_index = 0;
                }
                break;
                
            case STEP_LENGTH_LOW:
                data_length = byte;
                parse_state = STEP_LENGTH_HIGH;
                break;
                
            case STEP_LENGTH_HIGH:
                data_length |= (byte << 8);
                parse_state = STEP_SEQ;
                break;
                
            case STEP_SEQ:
                parse_state = STEP_CRC8;
                break;
                
            case STEP_CRC8:
                // For simplicity, we're not verifying CRC8 here
                parse_state = STEP_DATA;
                frame_length = ROS_PROTOCOL_HEADER_SIZE + ROS_PROTOCOL_CMD_SIZE + data_length + ROS_PROTOCOL_CRC16_SIZE;
                
                // Check if length is valid
                if (data_length > ROS_PROTOCOL_FRAME_MAX_SIZE - ROS_PROTOCOL_HEADER_SIZE - ROS_PROTOCOL_CMD_SIZE - ROS_PROTOCOL_CRC16_SIZE) {
                    RCLCPP_WARN(get_logger(), "Invalid frame length: %u", data_length);
                    buffer_index = 0;
                    parse_state = STEP_SOF;
                }
                break;
                
            case STEP_DATA:
                // Check if we have the complete frame
                if (buffer_index >= frame_length) {
                    // Process the complete frame
                    process_frame(frame_buffer.data(), frame_length);
                    frame_processed = true;
                    
                    // Reset state machine
                    buffer_index = 0;
                    parse_state = STEP_SOF;
                }
                break;
        }
        
        return frame_processed;
    }
    
    void process_frame(const uint8_t* frame, size_t length) {
        // Need at least header + cmd_id
        if (length < ROS_PROTOCOL_HEADER_SIZE + ROS_PROTOCOL_CMD_SIZE) {
            return;
        }
        
        // Extract cmd_id
        uint16_t cmd_id = frame[ROS_PROTOCOL_HEADER_SIZE] | (frame[ROS_PROTOCOL_HEADER_SIZE + 1] << 8);
        
        // Process based on cmd_id
        if (cmd_id == CHASSIS_STATUS_CMD_ID) {
            // Process encoder data
            size_t data_offset = ROS_PROTOCOL_HEADER_SIZE + ROS_PROTOCOL_CMD_SIZE;
            if (length >= data_offset + sizeof(chassis_encoder_data_t)) {
                const chassis_encoder_data_t* encoder_data = reinterpret_cast<const chassis_encoder_data_t*>(frame + data_offset);
                RCLCPP_INFO(get_logger(), "Encoder data: %d, %d, %d, %d", encoder_data->encoder_counts[0], encoder_data->encoder_counts[1], encoder_data->encoder_counts[2], encoder_data->encoder_counts[3]);
                update_odometry(encoder_data);
            }
        }
    }
    
    void update_odometry(const chassis_encoder_data_t* encoder_data) {
        rclcpp::Time now = this->now();
        static rclcpp::Time last_time = now;
        double dt = (now - last_time).seconds();
        last_time = now;
        
        if (dt <= 0.0) {
            return;
        }
        
        // Extract encoder counts
        int32_t delta_counts[4] = {0};
        
        for (int i = 0; i < 4; i++) {
            uint16_t current_counts = encoder_data->encoder_counts[i];
            
            // Initialize on first reading
            if (!encoder_initialized_[i]) {
                prev_encoder_counts_[i] = current_counts;
                encoder_initialized_[i] = true;
                continue;
            }
            
            // Calculate delta (handle wraparound)
            int32_t delta = static_cast<int32_t>(current_counts) - static_cast<int32_t>(prev_encoder_counts_[i]);
            if (delta > 32768) {
                delta -= 65536;
            } else if (delta < -32768) {
                delta += 65536;
            }
            
            delta_counts[i] = delta;
            prev_encoder_counts_[i] = current_counts;
        }
        
        // Mecanum wheel kinematics - assuming X configuration
        // FL: 0, FR: 1, RL: 2, RR: 3
        double wheel_fl_vel = delta_counts[0] * 2.0 * M_PI / encoder_counts_per_rev_ / dt;
        double wheel_fr_vel = delta_counts[1] * 2.0 * M_PI / encoder_counts_per_rev_ / dt;
        double wheel_rl_vel = delta_counts[2] * 2.0 * M_PI / encoder_counts_per_rev_ / dt;
        double wheel_rr_vel = delta_counts[3] * 2.0 * M_PI / encoder_counts_per_rev_ / dt;
        
        // Convert to robot velocities
        vx_ = wheel_radius_ * (wheel_fl_vel + wheel_fr_vel + wheel_rl_vel + wheel_rr_vel) / 4.0;
        vy_ = wheel_radius_ * (-wheel_fl_vel + wheel_fr_vel + wheel_rl_vel - wheel_rr_vel) / 4.0;
        vtheta_ = wheel_radius_ * (-wheel_fl_vel + wheel_fr_vel - wheel_rl_vel + wheel_rr_vel) / (4.0 * (wheel_base_ / 2.0));
        
        // Integrate odometry
        double delta_x = (vx_ * cos(theta_) - vy_ * sin(theta_)) * dt;
        double delta_y = (vx_ * sin(theta_) + vy_ * cos(theta_)) * dt;
        double delta_theta = vtheta_ * dt;
        
        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;
        
        // Normalize angle
        while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
    }
    
    void publish_odometry() {
        rclcpp::Time current_time = this->now();
        
        // // Prepare odometry message
        // nav_msgs::msg::Odometry odom;
        // odom.header.stamp = current_time;
        // odom.header.frame_id = frame_id_;
        // odom.child_frame_id = child_frame_id_;
        
        // // Set the position
        // odom.pose.pose.position.x = x_;
        // odom.pose.pose.position.y = y_;
        // odom.pose.pose.position.z = 0.0;
        
        // // Set the orientation
        // q.setRPY(0, 0, theta_);
        // odom.pose.pose.orientation.x = q.x();
        // odom.pose.pose.orientation.y = q.y();
        // odom.pose.pose.orientation.z = q.z();
        // odom.pose.pose.orientation.w = q.w();
        
        // // Set the velocity
        // odom.twist.twist.linear.x = vx_;
        // odom.twist.twist.linear.y = vy_;
        // odom.twist.twist.angular.z = vtheta_;
        
        // // Publish odometry
        // odom_pub_->publish(odom);
        
    }
    
    // Serial communication
    std::unique_ptr<serial::Serial> serial_;
    std::string serial_port_;
    int baud_rate_;
    
    // ROS2 publishers and timer
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    double wheel_radius_;
    double wheel_base_;
    double encoder_counts_per_rev_;
    double publish_rate_;
    std::string frame_id_;
    std::string child_frame_id_;
    
    // Odometry state
    double x_, y_, theta_;  // Position (meters) and orientation (radians)
    double vx_, vy_, vtheta_;  // Velocities
    
    // Encoder state
    uint16_t prev_encoder_counts_[4];
    bool encoder_initialized_[4];
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DjiEncoderReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 