# DJI 驱动节点

这个ROS2包提供了一个驱动节点，用于将ROS2标准的cmd_vel话题数据转换为DJI底盘可识别的串口命令，并接收编码器数据计算里程计信息。

## 功能特点

- 订阅cmd_vel话题并转换为DJI协议格式
- 支持麦克纳姆轮运动学模型
- 两路独立串口通信：一路发送控制命令，一路接收编码器数据
- 动态映射ROS Twist消息到DJI底盘364-1684范围
- 可选的协议调试功能，带频率控制

## 安装

确保您已经安装了ROS2和serial库：

```bash
sudo apt-get install ros-$ROS_DISTRO-serial
```

然后克隆仓库并编译：

```bash
cd ~/ros2_ws/src/
git clone https://github.com/your_username/dji_driver.git
cd ..
colcon build --packages-select dji_driver
```

## 使用方法

### 启动节点

```bash
# 使用默认参数
ros2 launch dji_driver dji_driver.launch.py

# 指定串口设备
ros2 launch dji_driver dji_driver.launch.py control_port:=/dev/ttyUSB0 encoder_port:=/dev/ttyUSB1

# 启用调试
ros2 launch dji_driver dji_driver.launch.py debug:=true protocol_debug:=true
```

### 发送控制命令

```bash
# 例如，向前移动
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

## 参数配置

在`config/dji_driver_params.yaml`中可以配置以下参数：

- 串口参数：设备名、波特率、超时时间
- 速度参数：偏移量、最大值
- 机器人物理参数：轮子半径、轮距、编码器分辨率等
- 调试参数：调试模式、协议调试、打印间隔

## 速度映射

本驱动程序将ROS2标准的速度命令（±max.x m/s, ±max.y m/s, ±max.z rad/s）映射到DJI底盘接受的364-1684范围：

- 1024: 停止
- 1684: 正向最大速度
- 364: 负向最大速度

## 注意事项

- 确保串口设备有正确的权限：`sudo chmod 666 /dev/ttyUSB*`
- 如果出现通信问题，尝试启用协议调试模式查看数据交换 