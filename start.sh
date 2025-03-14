#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/morefine/rm2025_hzu_sentry_ws/install/setup.bash

# source /opt/ros/humble/setup.bash
# source ~/ros_ws/install/setup.bash

# ros2 launch rm_vision_bringup opencv_nav.launch.py
# 启动所有进程
ros2 launch bot_navigation2 test_all.launch.py &
ros2 launch bot_navigation2 navigation2.launch.py use_sim_time:=False slam:=False map:=/home/morefine/rm2025_hzu_sentry_ws/slam_map.yaml &
ros2 run map_tools nav2_to_goal &
ros2 run map_tools cmd_vel2serial.py &
ros2 run map_tools odo &
ros2 run map_tools serial_port &
ros2 run map_tools clear_cost_map &

# 循环保持脚本运行
while true; do
    sleep 60
done
