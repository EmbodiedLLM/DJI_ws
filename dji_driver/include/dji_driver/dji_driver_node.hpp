#ifndef DJI_DRIVER_NODE_HPP_
#define DJI_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <nav_msgs/msg/odometry.hpp>
#include "dji_driver/dji_protocol.hpp"

namespace dji_driver
{

class DJIDriverNode : public rclcpp::Node
{
public:
    explicit DJIDriverNode();

private:
    // 初始化函数
    void init_parameters();
    bool init_serial();
    void init_subscriptions();
    void init_timers();

    // 回调函数
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void encoder_read_callback();
    void control_write_callback();

    // 串口读写函数
    bool read_encoder_data();
    bool write_control_data(const ControlCommand& cmd);
    void process_encoder_data(const EncoderData& data);

    // 参数
    // 控制串口参数
    std::string control_port_;
    int control_baud_rate_;
    double control_timeout_;
    // 编码器串口参数
    std::string encoder_port_;
    int encoder_baud_rate_;
    double encoder_timeout_;
    // 速度参数
    double vx_offset_, vy_offset_, wz_offset_;
    double vx_max_, vy_max_, wz_max_;
    bool debug_mode_;

    // ROS2相关
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr encoder_read_timer_;
    rclcpp::TimerBase::SharedPtr control_write_timer_;

    // 串口相关
    serial::Serial control_serial_;    // 用于发送控制命令的串口
    serial::Serial encoder_serial_;    // 用于接收编码器数据的串口
    bool control_connected_;
    bool encoder_connected_;
    ControlCommand current_cmd_;
    EncoderData latest_encoder_data_;

    void encoder_to_odom(const EncoderData& data); // 预留函数
};

} // namespace dji_driver

#endif // DJI_DRIVER_NODE_HPP_ 