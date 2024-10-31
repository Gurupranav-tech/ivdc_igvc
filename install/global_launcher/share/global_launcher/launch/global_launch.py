from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rplidar_ros_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py'
    )
    gps_ros_launch_file = os.path.join(
        get_package_share_directory('ublox_gps'), 'launch', 'ublox_gps_node_zedf9p-launch.py'
    )
    
    rplidar_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_ros_launch_file)
    )
    gps_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gps_ros_launch_file)
    )
    
    device_argument = DeclareLaunchArgument(
        'dev',
        default_value='/dev/ttyACM1',
        description='Device path for the serial connection'
    )
    
    return LaunchDescription([
        rplidar_ros_launch,    
        gps_ros_launch,
         device_argument,
        Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', LaunchConfiguration('dev')]
        ),
    ])