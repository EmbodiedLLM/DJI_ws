#include "dji_driver/dji_driver_node.hpp"
#include <nav_msgs/msg/odometry.hpp>

namespace dji_driver
{

/**
 * @brief 构造函数，初始化节点
 * 
 * 完成节点的初始化，包括：
 * 1. 加载参数
 * 2. 初始化串口连接
 * 3. 设置订阅发布器
 * 4. 创建定时器
 */
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

/**
 * @brief 初始化参数
 * 
 * 声明并获取节点运行所需的各项参数：
 * 1. 串口配置参数（端口、波特率、超时）
 * 2. 速度参数（偏移量、最大值）
 * 3. 调试模式开关
 */
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

/**
 * @brief 初始化串口连接
 * 
 * 根据配置参数初始化控制串口和编码器串口：
 * 1. 设置串口参数（端口名、波特率、超时）
 * 2. 尝试打开串口
 * 3. 记录连接状态
 * 
 * @return bool 返回初始化结果，如果任一串口初始化失败则返回false
 */
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

/**
 * @brief 初始化订阅和发布
 * 
 * 创建ROS2话题的订阅器和发布器：
 * 1. 订阅cmd_vel话题，接收速度控制命令
 * 2. 创建odom话题发布器，用于发布里程计信息
 */
void DJIDriverNode::init_subscriptions()
{
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&DJIDriverNode::cmd_vel_callback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
}

/**
 * @brief 初始化定时器
 * 
 * 创建定时器用于周期性执行任务：
 * 1. 编码器读取定时器：10ms周期，用于从串口读取编码器数据
 * 2. 控制命令发送定时器：50ms周期，用于向串口发送控制命令
 */
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

/**
 * @brief 处理cmd_vel话题的回调函数
 * 
 * 当接收到新的速度命令时：
 * 1. 应用偏移量和限制最大值
 * 2. 更新当前命令结构体
 * 3. 可选地输出调试信息
 * 
 * @param msg 接收到的Twist消息指针
 */
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

/**
 * @brief 编码器读取定时器回调函数
 * 
 * 定期执行以从串口读取编码器数据：
 * 1. 检查编码器串口连接状态
 * 2. 尝试读取编码器数据
 * 3. 处理读取到的数据
 */
void DJIDriverNode::encoder_read_callback()
{
    if (!encoder_connected_) return;
    
    if (read_encoder_data()) {
        process_encoder_data(latest_encoder_data_);
    }
}

/**
 * @brief 控制命令发送定时器回调函数
 * 
 * 定期执行以向串口发送控制命令：
 * 1. 检查控制串口连接状态
 * 2. 计算当前命令的校验和
 * 3. 发送控制命令
 */
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

/**
 * @brief 从串口读取编码器数据
 * 
 * 尝试从编码器串口读取数据并解析：
 * 1. 检查是否有足够的数据可读
 * 2. 读取并解析数据
 * 3. 验证数据有效性（帧头、帧尾、校验和）
 * 
 * @return bool 是否成功读取到有效数据
 */
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

/**
 * @brief 向串口发送控制数据
 * 
 * 将控制命令结构体写入串口：
 * 1. 尝试写入数据
 * 2. 验证是否写入成功
 * 3. 处理可能的异常
 * 
 * @param cmd 要发送的控制命令结构体
 * @return bool 是否发送成功
 */
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

/**
 * @brief 处理编码器数据
 * 
 * 对接收到的编码器数据进行处理：
 * 1. 可选地输出调试信息
 * 2. 调用编码器到里程计的转换函数
 * 
 * @param data 接收到的编码器数据结构体
 */
void DJIDriverNode::process_encoder_data(const EncoderData& data)
{
    if (debug_mode_) {
        RCLCPP_INFO(this->get_logger(), "Encoder data: fl=%d, fr=%d, rl=%d, rr=%d",
                    data.fl_encoder, data.fr_encoder, data.rl_encoder, data.rr_encoder);
    }
    // 预留：编码器数据转odom
    encoder_to_odom(data);
}

/**
 * @brief 将编码器数据转换为里程计信息
 * 
 * 根据编码器数据计算机器人位姿变化，并发布里程计消息。
 * 该函数需要根据实际机器人参数和运动学模型进行实现。
 * 
 * @param data 编码器数据结构体
 * 
 * @note 这是一个预留函数，用户需要根据实际机器人参数实现
 */
void DJIDriverNode::encoder_to_odom(const EncoderData& data)
{
    // TODO: 用户实现编码器到里程计的计算
    // 计算完成后发布odom
    // nav_msgs::msg::Odometry odom_msg;
    // odom_pub_->publish(odom_msg);
}

} // namespace dji_driver

/**
 * @brief 主函数
 * 
 * ROS2节点的入口点：
 * 1. 初始化ROS2
 * 2. 创建节点实例
 * 3. 进入事件循环
 * 4. 退出前清理资源
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出码
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dji_driver::DJIDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
