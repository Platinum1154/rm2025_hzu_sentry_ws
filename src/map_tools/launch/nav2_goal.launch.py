import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_tools',  # 包名
            executable='nav2_to_goal',  # 替换为你的可执行文件名
            name='nav_to_pose_client',  # 节点名称
            output='screen',  # 输出到屏幕
            parameters=[{
                # 这里可以添加你的参数
            }]
        )
    ])