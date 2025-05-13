// Copyright [2024] [Your Name]
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <serial/serial.h> // 来自 serial_driver 包

#include <string>
#include <vector>
#include <cmath> // M_PI 等
#include <chrono>
#include <algorithm> // For std::copy

#include "dji_receiver/protocol.h"
#include "dji_receiver/CRC8_CRC16.h"
#include <queue>

const std::chrono::seconds SERIAL_RECONNECT_DELAY(5); // 串口重连尝试的最小间隔

class DjiReceiverNode : public rclcpp::Node {
public:
    DjiReceiverNode() : Node("dji_receiver") {
        RCLCPP_INFO(this->get_logger(), "DJI Receiver 节点初始化中...");

        load_parameters();
        initialize_ros_entities();
        initialize_state();
        attempt_serial_connect(); // 首次尝试连接串口

        RCLCPP_INFO(this->get_logger(), "DJI Receiver 节点初始化完成。");
    }

    ~DjiReceiverNode() {
        if (serial_port_ptr_ && serial_port_ptr_->isOpen()) {
            try {
                serial_port_ptr_->close();
                RCLCPP_INFO(this->get_logger(), "串口已安全关闭。");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "关闭串口时发生异常: %s", e.what());
            }
        }
        RCLCPP_INFO(this->get_logger(), "DJI Receiver 节点已关闭。");
    }

private:


    // ROS Handles & Config
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string serial_port_;
    int baud_rate_;
    double publish_rate_;
    std::string frame_id_param_, child_frame_id_param_, robot_namespace_;
    std::string actual_frame_id_, actual_child_frame_id_, odom_topic_name_;
    std::vector<double> pose_covariance_, twist_covariance_;

    // State
    double acc_x_, acc_y_, acc_z_;
    rclcpp::Time last_update_time_;

    // Serial Port
    std::unique_ptr<serial::Serial> serial_port_ptr_;
    rclcpp::Time last_serial_reconnect_attempt_time_;

        
    // Add protocol unpack state
    unpack_data_t unpack_obj_;
    std::queue<uint8_t> rx_buffer_;
    

    // Initialize unpack state in initialize_state()
    void initialize_state() {
        acc_x_ = 0.0;
        acc_y_ = 0.0;
        acc_z_ = 0.0;
        last_update_time_ = this->now();
        last_serial_reconnect_attempt_time_ = this->now() - SERIAL_RECONNECT_DELAY; // 允许首次立即尝试连接
        
        // Initialize protocol unpack state
        memset(&unpack_obj_, 0, sizeof(unpack_data_t));
        unpack_obj_.p_header = (frame_header_struct_t *)unpack_obj_.protocol_packet;
        unpack_obj_.unpack_step = STEP_HEADER_SOF;
    }

    void load_parameters() {
        this->declare_parameter<std::string>("serial_port", "/dev/dji_serial");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<double>("publish_rate", 50.0);
        this->declare_parameter<std::string>("frame_id", "odom");
        this->declare_parameter<std::string>("child_frame_id", "base_link");
        this->declare_parameter<std::string>("robot_namespace", "");

        std::vector<double> default_pose_cov(36, 0.0);
        default_pose_cov[0] = default_pose_cov[7] = default_pose_cov[35] = 0.01; // x, y, yaw
        default_pose_cov[14] = default_pose_cov[21] = default_pose_cov[28] = 1e6; // z, roll, pitch (large)
        this->declare_parameter<std::vector<double>>("pose_covariance", default_pose_cov);

        std::vector<double> default_twist_cov(36, 0.0);
        default_twist_cov[0] = default_twist_cov[7] = default_twist_cov[35] = 0.005; // vx, vy, wz
        default_twist_cov[14] = default_twist_cov[21] = default_twist_cov[28] = 1e6; // vz, wx, wy (large)
        this->declare_parameter<std::vector<double>>("twist_covariance", default_twist_cov);

        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        frame_id_param_ = this->get_parameter("frame_id").as_string();
        child_frame_id_param_ = this->get_parameter("child_frame_id").as_string();
        robot_namespace_ = this->get_parameter("robot_namespace").as_string();

        pose_covariance_ = this->get_parameter("pose_covariance").as_double_array();
        twist_covariance_ = this->get_parameter("twist_covariance").as_double_array();

        if (pose_covariance_.size() != 36 || twist_covariance_.size() != 36) {
            RCLCPP_FATAL(this->get_logger(), "协方差矩阵大小无效！必须是36个元素。");
            if (rclcpp::ok()) rclcpp::shutdown();
            throw std::runtime_error("Invalid covariance matrix size");
        }

        // 处理命名空间
        if (!robot_namespace_.empty()) {
            if (robot_namespace_.back() == '/') robot_namespace_.pop_back();
            if (!robot_namespace_.empty() && robot_namespace_.front() == '/') {
                RCLCPP_WARN(this->get_logger(), "robot_namespace 不应以'/'开头，已移除。");
                robot_namespace_ = robot_namespace_.substr(1);
            }
        }
        
        actual_frame_id_ = robot_namespace_.empty() ? frame_id_param_ : robot_namespace_ + "/" + frame_id_param_;
        actual_child_frame_id_ = robot_namespace_.empty() ? child_frame_id_param_ : robot_namespace_ + "/" + child_frame_id_param_;
        odom_topic_name_ = robot_namespace_.empty() ? "odom" : robot_namespace_ + "/odom";

        RCLCPP_INFO(this->get_logger(), "串口: %s, 波特率: %d", serial_port_.c_str(), baud_rate_);
        RCLCPP_INFO(this->get_logger(), "发布频率: %.1f Hz, Odometry话题: %s", publish_rate_, odom_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "TF: %s -> %s", actual_frame_id_.c_str(), actual_child_frame_id_.c_str());
        if (!robot_namespace_.empty()) {
            RCLCPP_INFO(this->get_logger(), "机器人命名空间: %s", robot_namespace_.c_str());
        }
    }

    void initialize_ros_entities() {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        if (publish_rate_ <= 0) {
            RCLCPP_FATAL(this->get_logger(), "发布频率 (publish_rate) 必须大于0。");
            if (rclcpp::ok()) rclcpp::shutdown();
            throw std::runtime_error("Invalid publish_rate");
        }
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(timer_period, std::bind(&DjiReceiverNode::timer_callback, this));
    }


    void timer_callback() {
        chassis_move_t received_data;
        bool data_available = false;

        data_available = receive_chassis_data_from_serial(received_data);
       

        if (data_available) {
            update_and_publish_odometry(received_data);
        } else {
            attempt_serial_reconnect_if_needed();
        }
    }

   bool receive_chassis_data_from_serial(chassis_move_t& data) {
        if (!serial_port_ptr_ || !serial_port_ptr_->isOpen()) {
            return false;
        }
        
        try {
            // Read available data and add to buffer
            if (serial_port_ptr_->available() > 0) {
                size_t bytes_available = serial_port_ptr_->available();
                std::vector<uint8_t> temp_buffer(bytes_available);
                serial_port_ptr_->read(temp_buffer.data(), bytes_available);
                
                // Add to queue
                for (uint8_t byte : temp_buffer) {
                    rx_buffer_.push(byte);
                }
            }
            
            // Process bytes in the buffer
            bool packet_unpacked = false;
            while (!rx_buffer_.empty() && !packet_unpacked) {
                uint8_t byte = rx_buffer_.front();
                rx_buffer_.pop();
                
                packet_unpacked = protocol_unpack_byte(byte);
                
                if (packet_unpacked) {
                    // Extract command ID
                    uint16_t cmd_id = *(uint16_t *)(unpack_obj_.protocol_packet + ROS_PROTOCOL_HEADER_SIZE);
                    
                    if (cmd_id == CHASSIS_STATUS_CMD_ID) {
                        // Extract chassis data (starts after header + cmd_id)
                        chassis_upload_data_t* chassis_data = (chassis_upload_data_t*)(
                            unpack_obj_.protocol_packet + ROS_PROTOCOL_HEADER_SIZE + ROS_PROTOCOL_CMD_SIZE);
                        
                        // Convert the protocol data to chassis_move_t
                        data.vx = chassis_data->vx.f;
                        data.vy = chassis_data->vy.f;
                        data.wx = 0.0;  // Not in protocol
                        data.wy = 0.0;  // Not in protocol
                        data.wz = chassis_data->wz.f;
                        data.roll = chassis_data->chassis_roll.f;
                        data.pitch = chassis_data->chassis_pitch.f;
                        data.yaw = chassis_data->chassis_yaw.f;
                        
                        return true;
                    }
                }
            }
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "串口读取IO异常: %s. 将关闭并尝试重连。", e.what());
            if(serial_port_ptr_ && serial_port_ptr_->isOpen()) serial_port_ptr_->close();
            serial_port_ptr_.reset(); // 强制重连
            last_serial_reconnect_attempt_time_ = this->now() - SERIAL_RECONNECT_DELAY; // 允许立即重连
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "串口读取时发生未知异常: %s", e.what());
        }
        
        return false;
    }


    void update_and_publish_odometry(const chassis_move_t& chassis_data) {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_update_time_).seconds();

        if (dt <= 1e-9) { // 避免 dt 为零或负数，允许非常小的时间间隔
            if (dt < 0.0) {
                RCLCPP_WARN(this->get_logger(), "检测到时间倒流 (dt = %.4f s)。重置上次更新时间。", dt);
                last_update_time_ = current_time;
            }
            // 若dt过小，则可能无需积分，但仍可发布当前速度/姿态
            // 为简化，此处若dt非正，则不进行更新。实际应用中可能需要更细致处理。
            return;
        }

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = actual_frame_id_;
        odom_msg.child_frame_id = actual_child_frame_id_;

        odom_msg.twist.twist.linear.x = chassis_data.vx;
        odom_msg.twist.twist.linear.y = chassis_data.vy;
        odom_msg.twist.twist.linear.z = chassis_data.vz;
        odom_msg.twist.twist.angular.x = chassis_data.wx;
        odom_msg.twist.twist.angular.y = chassis_data.wy;
        odom_msg.twist.twist.angular.z = chassis_data.wz;

        tf2::Quaternion q_orientation;
        q_orientation.setRPY(chassis_data.roll, chassis_data.pitch, chassis_data.yaw);
        odom_msg.pose.pose.orientation = tf2::toMsg(q_orientation);
        
        // 位置积分: 将机器人坐标系下的速度转换到世界坐标系下累加
        double current_yaw = chassis_data.yaw;
        acc_x_ += (chassis_data.vx * std::cos(current_yaw) - chassis_data.vy * std::sin(current_yaw)) * dt;
        acc_y_ += (chassis_data.vx * std::sin(current_yaw) + chassis_data.vy * std::cos(current_yaw)) * dt;
        acc_z_ += chassis_data.vz * dt; // 如果vz有效

        odom_msg.pose.pose.position.x = acc_x_;
        odom_msg.pose.pose.position.y = acc_y_;
        odom_msg.pose.pose.position.z = acc_z_;

        std::copy(pose_covariance_.begin(), pose_covariance_.end(), odom_msg.pose.covariance.begin());
        std::copy(twist_covariance_.begin(), twist_covariance_.end(), odom_msg.twist.covariance.begin());

        odom_pub_->publish(odom_msg);

        geometry_msgs::msg::TransformStamped tf_transform;
        tf_transform.header.stamp = current_time;
        tf_transform.header.frame_id = actual_frame_id_;
        tf_transform.child_frame_id = actual_child_frame_id_;
        tf_transform.transform.translation.x = acc_x_;
        tf_transform.transform.translation.y = acc_y_;
        tf_transform.transform.translation.z = acc_z_;
        tf_transform.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_transform);

        last_update_time_ = current_time;
    }

    void attempt_serial_connect() {
        RCLCPP_INFO(this->get_logger(), "尝试连接串口 %s (波特率 %d)...", serial_port_.c_str(), baud_rate_);
        try {
            serial_port_ptr_ = std::make_unique<serial::Serial>(
                serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
            if (serial_port_ptr_->isOpen()) {
                RCLCPP_INFO(this->get_logger(), "串口 %s 打开成功。", serial_port_.c_str());
                serial_port_ptr_->flushInput();
            } else {
                RCLCPP_WARN(this->get_logger(), "串口 %s 创建但未打开。(权限或设备问题?)", serial_port_.c_str());
                serial_port_ptr_.reset();
            }
        } catch (const serial::IOException& e) {
            RCLCPP_WARN(this->get_logger(), "打开串口 %s IO异常: %s", serial_port_.c_str(), e.what());
            serial_port_ptr_.reset();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "打开串口 %s 时发生未知异常: %s", serial_port_.c_str(), e.what());
            serial_port_ptr_.reset();
        }
        last_serial_reconnect_attempt_time_ = this->now();
    }

    void attempt_serial_reconnect_if_needed() {
        if (serial_port_ptr_ && serial_port_ptr_->isOpen()) return;

        if ((this->now() - last_serial_reconnect_attempt_time_) >= SERIAL_RECONNECT_DELAY) {
            RCLCPP_INFO(this->get_logger(), "串口未连接或已断开，尝试重新连接...");
            attempt_serial_connect();
        }
    }


    /**
     * @brief Process a single byte according to the protocol
     * @param byte The byte to process
     * @return true if a complete packet was successfully unpacked
     */
    bool protocol_unpack_byte(uint8_t byte) {
        switch (unpack_obj_.unpack_step) {
            case STEP_HEADER_SOF:
                if (byte == HEADER_SOF) {
                    unpack_obj_.unpack_step = STEP_LENGTH_LOW;
                    unpack_obj_.protocol_packet[0] = byte;
                }
                break;
            
            case STEP_LENGTH_LOW:
                unpack_obj_.data_len = byte;
                unpack_obj_.protocol_packet[1] = byte;
                unpack_obj_.unpack_step = STEP_LENGTH_HIGH;
                break;
            
            case STEP_LENGTH_HIGH:
                unpack_obj_.data_len |= (byte << 8);
                unpack_obj_.protocol_packet[2] = byte;
                
                if (unpack_obj_.data_len < sizeof(chassis_upload_data_t) || 
                    unpack_obj_.data_len > ROS_PROTOCOL_FRAME_MAX_SIZE - ROS_HEADER_CRC_CMDID_LEN) {
                    unpack_obj_.unpack_step = STEP_HEADER_SOF;
                    RCLCPP_WARN(this->get_logger(), "Invalid data length: %u", unpack_obj_.data_len);
                } else {
                    unpack_obj_.unpack_step = STEP_FRAME_SEQ;
                }
                break;
            
            case STEP_FRAME_SEQ:
                unpack_obj_.protocol_packet[3] = byte;
                unpack_obj_.unpack_step = STEP_HEADER_CRC8;
                break;
            
            case STEP_HEADER_CRC8:
                unpack_obj_.protocol_packet[4] = byte;
                
                // Verify header CRC8
                if (verify_CRC8_check_sum(unpack_obj_.protocol_packet, ROS_PROTOCOL_HEADER_SIZE)) {
                    unpack_obj_.unpack_step = STEP_DATA_CRC16;
                    unpack_obj_.index = ROS_PROTOCOL_HEADER_SIZE;
                } else {
                    unpack_obj_.unpack_step = STEP_HEADER_SOF;
                    RCLCPP_WARN(this->get_logger(), "Header CRC8 verification failed");
                }
                break;
            
            case STEP_DATA_CRC16:
                if (unpack_obj_.index < (ROS_HEADER_CRC_CMDID_LEN + unpack_obj_.data_len)) {
                    unpack_obj_.protocol_packet[unpack_obj_.index++] = byte;
                }
                
                // Check if we've received the complete packet
                if (unpack_obj_.index >= (ROS_HEADER_CRC_CMDID_LEN + unpack_obj_.data_len)) {
                    // Verify the entire packet with CRC16
                    if (verify_CRC16_check_sum(unpack_obj_.protocol_packet, unpack_obj_.index)) {
                        uint16_t cmd_id = *(uint16_t *)(unpack_obj_.protocol_packet + ROS_PROTOCOL_HEADER_SIZE);
                        
                        // Reset for next packet
                        unpack_obj_.unpack_step = STEP_HEADER_SOF;
                        
                        // Return true to indicate successful unpack if this is chassis data
                        return (cmd_id == CHASSIS_STATUS_CMD_ID);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Packet CRC16 verification failed");
                        unpack_obj_.unpack_step = STEP_HEADER_SOF;
                    }
                }
                break;
            
            default:
                unpack_obj_.unpack_step = STEP_HEADER_SOF;
                break;
        }
        
        return false;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    try {
      auto node = std::make_shared<DjiReceiverNode>();
      rclcpp::spin(node);
    } catch (const std::exception& e) {
        fprintf(stderr, "节点初始化或运行时捕获到异常: %s\n", e.what());
        // 确保即使在构造函数中抛出异常，rclcpp也能正确关闭
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }
    if (rclcpp::ok()) { // 如果 spin 正常退出 (例如因为 Ctrl+C)
        rclcpp::shutdown();
    }
    return 0;
} 