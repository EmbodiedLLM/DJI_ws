#include "dji_driver/dji_driver_node.hpp"
#include <nav_msgs/msg/odometry.hpp>

namespace dji_driver
{

DJIDriverNode::DJIDriverNode()
: Node("dji_driver_node"), control_connected_(false), encoder_connected_(false)
{
    // 初始化参数
    init_parameters();
    
    // 初始化串口
    if (!init_serial()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial ports");
        // 即使串口初始化失败，节点仍然继续运行，可能后续会自动重连
    }
    
    // 初始化订阅和定时器
    init_subscriptions();
    init_timers();
    
    RCLCPP_INFO(this->get_logger(), "DJIDriverNode has been initialized");
}

void DJIDriverNode::init_parameters()
{
    // 读取控制串口参数
    this->declare_parameter("control_serial.port", "/dev/ttyUSB0");
    this->declare_parameter("control_serial.baud_rate", 115200);
    this->declare_parameter("control_serial.timeout", 0.1);
    
    // 读取编码器串口参数
    this->declare_parameter("encoder_serial.port", "/dev/ttyUSB1");
    this->declare_parameter("encoder_serial.baud_rate", 115200);
    this->declare_parameter("encoder_serial.timeout", 0.1);
    
    // 读取速度参数
    this->declare_parameter("velocity.linear.x.offset", 0.0);
    this->declare_parameter("velocity.linear.x.max", 1.0);
    this->declare_parameter("velocity.linear.y.offset", 0.0);
    this->declare_parameter("velocity.linear.y.max", 1.0);
    this->declare_parameter("velocity.angular.z.offset", 0.0);
    this->declare_parameter("velocity.angular.z.max", 1.0);
    
    // 读取其他参数
    this->declare_parameter("debug", false);
    
    // 获取控制串口参数值
    control_port_ = this->get_parameter("control_serial.port").as_string();
    control_baud_rate_ = this->get_parameter("control_serial.baud_rate").as_int();
    control_timeout_ = this->get_parameter("control_serial.timeout").as_double();
    
    // 获取编码器串口参数值
    encoder_port_ = this->get_parameter("encoder_serial.port").as_string();
    encoder_baud_rate_ = this->get_parameter("encoder_serial.baud_rate").as_int();
    encoder_timeout_ = this->get_parameter("encoder_serial.timeout").as_double();
    
    // 获取速度参数值
    vx_offset_ = this->get_parameter("velocity.linear.x.offset").as_double();
    vx_max_ = this->get_parameter("velocity.linear.x.max").as_double();
    vy_offset_ = this->get_parameter("velocity.linear.y.offset").as_double();
    vy_max_ = this->get_parameter("velocity.linear.y.max").as_double();
    wz_offset_ = this->get_parameter("velocity.angular.z.offset").as_double();
    wz_max_ = this->get_parameter("velocity.angular.z.max").as_double();
    
    debug_mode_ = this->get_parameter("debug").as_bool();
}

bool DJIDriverNode::init_serial()
{
    bool success = true;
    
    // 初始化控制串口
    try {
        control_serial_.setPort(control_port_);
        control_serial_.setBaudrate(control_baud_rate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(control_timeout_ * 1000));
        control_serial_.setTimeout(timeout);
        control_serial_.open();
        control_connected_ = true;
        RCLCPP_INFO(this->get_logger(), "Control serial port %s opened successfully", control_port_.c_str());
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open control serial port: %s", e.what());
        success = false;
    }
    
    // 初始化编码器串口
    try {
        encoder_serial_.setPort(encoder_port_);
        encoder_serial_.setBaudrate(encoder_baud_rate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(encoder_timeout_ * 1000));
        encoder_serial_.setTimeout(timeout);
        encoder_serial_.open();
        encoder_connected_ = true;
        RCLCPP_INFO(this->get_logger(), "Encoder serial port %s opened successfully", encoder_port_.c_str());
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open encoder serial port: %s", e.what());
        success = false;
    }
    
    return success;
}

void DJIDriverNode::init_subscriptions()
{
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&DJIDriverNode::cmd_vel_callback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
}

void DJIDriverNode::init_timers()
{
    // 创建定时器用于读取编码器数据
    encoder_read_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&DJIDriverNode::encoder_read_callback, this));
    
    // 创建定时器用于发送控制命令
    control_write_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DJIDriverNode::control_write_callback, this));
}

void DJIDriverNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // 应用偏移量和限制最大值
    current_cmd_.vx = std::clamp(msg->linear.x + vx_offset_, -vx_max_, vx_max_);
    current_cmd_.vy = std::clamp(msg->linear.y + vy_offset_, -vy_max_, vy_max_);
    current_cmd_.wz = std::clamp(msg->angular.z + wz_offset_, -wz_max_, wz_max_);
    
    if (debug_mode_) {
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: vx=%.2f, vy=%.2f, wz=%.2f",
                    current_cmd_.vx, current_cmd_.vy, current_cmd_.wz);
    }
}

void DJIDriverNode::encoder_read_callback()
{
    if (!encoder_connected_) return;
    
    if (read_encoder_data()) {
        process_encoder_data(latest_encoder_data_);
    }
}

void DJIDriverNode::control_write_callback()
{
    if (!control_connected_) return;
    
    // ===== [协议相关] 计算校验和 =====
    // 注意：根据实际协议调整校验和计算方法
    current_cmd_.checksum = calculate_checksum(
        reinterpret_cast<uint8_t*>(&current_cmd_),
        sizeof(ControlCommand) - 2);  // 减去校验和和帧尾的长度
    // ===================================
    
    write_control_data(current_cmd_);
}

bool DJIDriverNode::read_encoder_data()
{
    // ===== [协议相关] 判断可用数据大小 =====
    if (encoder_serial_.available() < sizeof(EncoderData)) {
        return false;
    }
    // ======================================
    
    try {
        // ===== [协议相关] 数据读取和解析 =====
        std::vector<uint8_t> buffer(sizeof(EncoderData));
        size_t bytes_read = encoder_serial_.read(buffer.data(), buffer.size());
        
        if (bytes_read == sizeof(EncoderData)) {
            memcpy(&latest_encoder_data_, buffer.data(), sizeof(EncoderData));
            
            // 验证帧头和帧尾
            if (latest_encoder_data_.header == 0xBB && latest_encoder_data_.footer == 0x55) {
                // 验证校验和
                uint8_t calculated_checksum = calculate_checksum(
                    buffer.data(),
                    sizeof(EncoderData) - 2);  // 减去校验和和帧尾的长度
                
                if (calculated_checksum == latest_encoder_data_.checksum) {
                    return true;
                }
            }
        }
        // ======================================
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Encoder serial read error: %s", e.what());
        encoder_connected_ = false;
    }
    
    return false;
}

bool DJIDriverNode::write_control_data(const ControlCommand& cmd)
{
    try {
        // ===== [协议相关] 发送控制命令 =====
        size_t bytes_written = control_serial_.write(
            reinterpret_cast<const uint8_t*>(&cmd),
            sizeof(ControlCommand));
        return bytes_written == sizeof(ControlCommand);
        // ===================================
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Control serial write error: %s", e.what());
        control_connected_ = false;
        return false;
    }
}

void DJIDriverNode::process_encoder_data(const EncoderData& data)
{
    if (debug_mode_) {
        RCLCPP_INFO(this->get_logger(), "Encoder data: fl=%d, fr=%d, rl=%d, rr=%d",
                    data.fl_encoder, data.fr_encoder, data.rl_encoder, data.rr_encoder);
    }
    // 预留：编码器数据转odom
    encoder_to_odom(data);
}

void DJIDriverNode::encoder_to_odom(const EncoderData& data)
{
    // TODO: 用户实现编码器到里程计的计算
    // 计算完成后发布odom
    // nav_msgs::msg::Odometry odom_msg;
    // odom_pub_->publish(odom_msg);
}

} // namespace dji_driver

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dji_driver::DJIDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
