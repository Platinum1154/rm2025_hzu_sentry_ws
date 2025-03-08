source ~/rm2025_hzu_sentry_ws/install/setup.bash && sleep 3 && \
nohup ros2 launch bot_navigation2 test_all.launch.py > /dev/null 2>&1 & echo 1 && sleep 3 && \
nohup ros2 launch bot_navigation2 navigation2.launch.py use_sim_time:=False slam:=False map:=/home/morefine/rm2025_hzu_sentry_ws/slam_map.yaml > /dev/null 2>&1 & echo 3 && sleep 3 && \
nohup ros2 run map_tools nav2_to_goal > /dev/null 2>&1 & echo 4 && sleep 3 && \
nohup ros2 run map_tools cmd_vel2serial.py > /dev/null 2>&1 & echo 5 && sleep 3 && \
nohup ros2 run map_tools odo > /dev/null 2>&1 & echo 2 && sleep 3 && \
nohup ros2 run map_tools serial_port > /dev/null 2>&1 & echo 6 && sleep 3 && \
nohup ros2 run map_tools clear_cost_map > /dev/null 2>&1 & echo 7
