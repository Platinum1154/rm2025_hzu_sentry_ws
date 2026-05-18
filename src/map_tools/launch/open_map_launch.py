from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os
import launch

def generate_launch_description():
    # 获取地图文件路径
    map_file_path = os.path.join(
        get_package_share_directory('map_tools'),  # 包名
        'maps',  # 地图文件夹
        'slam_map.yaml'  # 地图yaml文件名
    )

    # map_server lifecycle node
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',  # namespace 设置为空字符串
        output='screen',
        parameters=[{'yaml_filename': map_file_path}],
    )

    # 自动激活 map_server 的事件处理
    return LaunchDescription([
        map_server_node,
        LogInfo(msg="Map server node started, waiting for activation..."),
        RegisterEventHandler(
            OnProcessExit(
                target_action=map_server_node,
                on_exit=[
                    LogInfo(msg="Map server process exited, activating node..."),
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=launch.events.matches_action(map_server_node),
                            transition_id=Transition.TRANSITION_ACTIVATE
                        )
                    )
                ]
            )
        ),
    ])
