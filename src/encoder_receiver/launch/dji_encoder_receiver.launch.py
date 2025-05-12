from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM2')
    baud_rate = LaunchConfiguration('baud_rate', default='115200')
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.076')
    wheel_base = LaunchConfiguration('wheel_base', default='0.4')
    encoder_counts_per_rev = LaunchConfiguration('encoder_counts_per_rev', default='8192.0')
    publish_rate = LaunchConfiguration('publish_rate', default='50.0')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM2',
            description='Serial port for DJI encoder data'
        ),
        
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for serial communication'
        ),
        
        DeclareLaunchArgument(
            'wheel_radius',
            default_value='0.076',
            description='Wheel radius in meters'
        ),
        
        DeclareLaunchArgument(
            'wheel_base',
            default_value='0.4',
            description='Wheel base (distance between wheels) in meters'
        ),
        
        DeclareLaunchArgument(
            'encoder_counts_per_rev',
            default_value='8192.0',
            description='Encoder counts per revolution'
        ),
        
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Odometry publish rate in Hz'
        ),
        
        # Node
        Node(
            package='encoder_receiver',
            executable='dji_encoder_receiver',
            name='dji_encoder_receiver',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'baud_rate': baud_rate,
                'wheel_radius': wheel_radius,
                'wheel_base': wheel_base,
                'encoder_counts_per_rev': encoder_counts_per_rev,
                'publish_rate': publish_rate,
                'frame_id': 'odom',
                'child_frame_id': 'base_link'
            }]
        )
    ]) 