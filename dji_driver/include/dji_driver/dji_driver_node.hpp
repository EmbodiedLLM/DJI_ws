#ifndef DJI_DRIVER_NODE_HPP_
#define DJI_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include "dji_driver/dji_protocol.hpp"

namespace dji_driver
{

/**
 * @brief DJI麦克纳姆轮底盘驱动节点类
 * 
 * 该节点提供以下功能：
 * 1. 订阅cmd_vel话题，将速度命令转换为底盘控制指令
 * 2. 通过两个独立串口与底盘通信：一个发送控制指令，一个接收编码器数据
 * 3. 根据编码器数据计算里程计信息并发布odom话题
 */
class DJIDriverNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * 
     * 初始化节点，包括参数加载、串口初始化、订阅发布设置和定时器创建
     */
    explicit DJIDriverNode();

private:
    /**
     * @brief 初始化所有节点参数
     * 
     * 声明并获取节点运行所需的各项参数，包括串口配置、速度参数等
     */
    void init_parameters();
    
    /**
     * @brief 初始化串口连接
     * 
     * 根据配置参数初始化控制串口和编码器串口
     * @return bool 初始化是否成功，有任一串口失败则返回false
     */
    bool init_serial();
    
    /**
     * @brief 初始化订阅发布器
     * 
     * 创建cmd_vel的订阅及odom的发布器
     */
    void init_subscriptions();
    
    /**
     * @brief 初始化定时器
     * 
     * 创建用于定期读取编码器和发送控制命令的定时器
     */
    void init_timers();

    /**
     * @brief cmd_vel话题回调函数
     * 
     * 处理接收到的速度控制命令，应用速度限制和偏移量
     * @param msg 接收到的Twist消息指针
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    /**
     * @brief 编码器串口读取定时回调
     * 
     * 定期从编码器串口读取数据，解析后处理编码器数据
     */
    void encoder_read_callback();
    
    /**
     * @brief 控制指令发送定时回调
     * 
     * 定期向控制串口发送最新的控制命令
     */
    void control_write_callback();

    /**
     * @brief 从串口读取编码器数据
     * 
     * 读取并解析编码器数据，验证数据有效性
     * @return bool 是否成功读取到有效数据
     */
    bool read_encoder_data();
    
    /**
     * @brief 向串口发送控制数据
     * 
     * 将控制命令结构体写入串口
     * @param cmd 要发送的控制命令
     * @return bool 发送是否成功
     */
    bool write_control_data(const ControlCommand& cmd);
    
    /**
     * @brief 处理编码器数据
     * 
     * 对解析得到的编码器数据进行处理，包括调试输出和里程计计算
     * @param data 接收到的编码器数据结构体
     */
    void process_encoder_data(const EncoderData& data);

    /**
     * @brief 将编码器数据转换为里程计信息
     * 
     * 根据编码器数据计算机器人位姿变化，并发布里程计消息
     * @param data 编码器数据结构体
     */
    void encoder_to_odom(const EncoderData& data);
    
    /**
     * @brief 发布TF变换
     * 
     * 发布从odom到base_link的TF变换
     * @param x X方向坐标
     * @param y Y方向坐标
     * @param theta 航向角
     * @param stamp 时间戳
     */
    void publish_odom_tf(double x, double y, double theta, const rclcpp::Time& stamp);

    // 控制串口参数
    std::string control_port_;          ///< 控制串口设备名
    int control_baud_rate_;             ///< 控制串口波特率
    double control_timeout_;            ///< 控制串口读写超时时间(秒)
    
    // 编码器串口参数
    std::string encoder_port_;          ///< 编码器串口设备名
    int encoder_baud_rate_;             ///< 编码器串口波特率
    double encoder_timeout_;            ///< 编码器串口读写超时时间(秒)
    
    // 速度参数
    double vx_offset_;                  ///< X方向速度偏移量
    double vy_offset_;                  ///< Y方向速度偏移量
    double wz_offset_;                  ///< Z轴角速度偏移量
    double vx_max_;                     ///< X方向最大速度限制
    double vy_max_;                     ///< Y方向最大速度限制
    double wz_max_;                     ///< Z轴最大角速度限制
    
    // 机器人物理参数
    double wheel_radius_;               ///< 轮子半径(m)
    double wheel_distance_x_;           ///< 轴距(m)
    double wheel_distance_y_;           ///< 轮距(m)
    int encoder_resolution_;            ///< 编码器分辨率(脉冲/转)
    double mecanum_angle_rad_;          ///< 麦克纳姆轮安装角度(弧度)
    
    // 里程计参数
    std::string odom_frame_id_;         ///< 里程计坐标系名称
    std::string base_frame_id_;         ///< 机器人坐标系名称
    bool publish_tf_;                   ///< 是否发布TF变换
    
    bool debug_mode_;                   ///< 调试模式开关
    bool protocol_debug_;               ///< 协议调试模式开关，用于打印16进制协议数据
    double debug_print_interval_;       ///< 调试信息打印间隔(秒)
    rclcpp::Time last_tx_print_time_;   ///< 上次发送数据打印时间
    rclcpp::Time last_rx_print_time_;   ///< 上次接收数据打印时间

    // ROS2相关
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;      ///< cmd_vel话题订阅器
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;              ///< odom话题发布器
    rclcpp::TimerBase::SharedPtr encoder_read_timer_;                             ///< 编码器读取定时器
    rclcpp::TimerBase::SharedPtr control_write_timer_;                            ///< 控制指令发送定时器
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;               ///< TF变换广播器

    // 串口相关
    serial::Serial control_serial_;     ///< 用于发送控制命令的串口对象
    serial::Serial encoder_serial_;     ///< 用于接收编码器数据的串口对象
    bool control_connected_;            ///< 控制串口连接状态标志
    bool encoder_connected_;            ///< 编码器串口连接状态标志
    ControlCommand current_cmd_;        ///< 当前的控制命令
    EncoderData latest_encoder_data_;   ///< 最新接收到的编码器数据
    
    // 里程计计算相关
    double x_, y_, theta_;              ///< 当前位姿
    double vx_, vy_, vtheta_;           ///< 当前速度
    int32_t last_fl_encoder_;           ///< 上次左前轮编码器值
    int32_t last_fr_encoder_;           ///< 上次右前轮编码器值
    int32_t last_rl_encoder_;           ///< 上次左后轮编码器值
    int32_t last_rr_encoder_;           ///< 上次右后轮编码器值
    rclcpp::Time last_encoder_time_;    ///< 上次编码器数据时间戳
    bool first_encoder_received_;       ///< 是否收到第一帧编码器数据
};

} // namespace dji_driver

#endif // DJI_DRIVER_NODE_HPP_ 