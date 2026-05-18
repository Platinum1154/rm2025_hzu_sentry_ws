from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource,AnyLaunchDescriptionSource
import lifecycle_msgs.msg
import os


def generate_launch_description():


    node_name = 'merger_node'

    lidar_x3_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch'),
        '/x3_ydlidar_launch.py']),
    )
    lidar_x3_launch_2 = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch'),
        '/x3_ydlidar_launch.py']),
    )

    merged_scan = LifecycleNode(package='map_tools',
                                executable='merger_node',
                                name='merger_node',
                                namespace='/',
                                output='screen',
                                emulate_tty=True)

    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser_merged',
                    arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'merged_laser'],
                    )

    return LaunchDescription([
        merged_scan,
        tf2_node,
        # lidar_x3_launch,
        # lidar_x3_launch_2
    ])

# 两个雷达不能同时启动，报的错误是2雷达的串口加载成默认串口ydlidar  已解决