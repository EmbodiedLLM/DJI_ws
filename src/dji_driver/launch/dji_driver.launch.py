from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取参数文件的路径
    config_file = os.path.join(
        get_package_share_directory('dji_driver'),
        'config',
        'dji_driver_params.yaml'
    )
    
    # 声明命令行参数，用于覆盖配置文件中的值
    control_port_arg = DeclareLaunchArgument(
        'control_port', 
        default_value='/dev/ttyACM0',
        description='Control serial port device'
    )
    
    encoder_port_arg = DeclareLaunchArgument(
        'encoder_port', 
        default_value='/dev/ttyACM2',
        description='Encoder serial port device'
    )
    
    # 创建节点启动配置
    dji_driver_node = Node(
        package='dji_driver',
        executable='dji_driver_node',
        name='dji_driver',
        output='screen',
        parameters=[
            config_file,
            {
                'control_serial.port': LaunchConfiguration('control_port'),
                'encoder_serial.port': LaunchConfiguration('encoder_port'),
            }
        ],
    )
    
    # 返回启动描述
    return LaunchDescription([
        control_port_arg,
        encoder_port_arg,
        dji_driver_node
    ]) 