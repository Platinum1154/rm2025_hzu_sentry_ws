#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')

    # 定义参数文件的 LaunchArgument
    # params_declare = DeclareLaunchArgument('params_file_1',
    #                                        default_value=os.path.join(
    #                                            share_dir, 'params', 'ydlidar_x3.yaml'),
    #                                        description='Path to the ROS2 parameters file to use.')

    params_declare_2 = DeclareLaunchArgument('params_file_2',
                                             default_value=os.path.join(
                                                 share_dir, 'params', 'ydlidar_x3_2.yaml'),
                                             description='Path to the ROS2 parameters file to use.')

    # 激光雷达1的节点
    # driver_node_1 = LifecycleNode(package='ydlidar_ros2_driver',
    #                                executable='ydlidar_ros2_driver_node',
    #                                name='ydlidar_ros2_driver_node_1',
    #                                namespace='/',
    #                                output='screen',
    #                                emulate_tty=True,
    #                                parameters=[LaunchConfiguration('params_file_1')])

    # 激光雷达2的节点
    driver_node_2 = LifecycleNode(package='ydlidar_ros2_driver',
                                   executable='ydlidar_ros2_driver_node',
                                   name='ydlidar_ros2_driver_node_2',
                                   namespace='/',
                                   output='screen',
                                   emulate_tty=True,
                                   parameters=[LaunchConfiguration('params_file_2')])

    # 静态变换发布器，发布激光雷达2的变换
    tf2_node_laser2 = Node(package='tf2_ros',
                            executable='static_transform_publisher',
                            name='static_tf_pub_laser_2',
                            arguments=['0.11192', '0.10962', '0', '0', '0', '1', '0', 'base_link', 'laser2'])

    # # 静态变换发布器，发布激光雷达1的变换
    # tf2_node_laser1 = Node(package='tf2_ros',
    #                         executable='static_transform_publisher',
    #                         name='static_tf_pub_laser_1',
    #                         arguments=['-0.115', '0', '0', '0', '0', '0', '1', 'base_link', 'laser1'])

    return LaunchDescription([
        # params_declare,
        params_declare_2,
        # driver_node_1,
        driver_node_2,
        tf2_node_laser2,
        # tf2_node_laser1,
    ])
