
#!/bin/bash

echo "Sourcing ROS 2 environment..."
source /opt/ros/humble/setup.bash
source /home/morefine/rm2025_hzu_sentry_ws/install/setup.bash

echo "Launching processes in separate terminals..."

gnome-terminal --tab --title="test_all.launch.py" -- bash -c "ros2 launch bot_navigation2 test_all.launch.py; exec bash"
gnome-terminal --tab --title="navigation2_copy.launch.py" -- bash -c "ros2 launch bot_navigation2 navigation2_copy.launch.py use_sim_time:=False slam:=False map:=/home/morefine/rm2025_hzu_sentry_ws/slam_map.yaml; exec bash"
gnome-terminal --tab --title="nav2_to_goal" -- bash -c "ros2 run map_tools nav2_to_goal; exec bash"
gnome-terminal --tab --title="cmd_vel2serial.py" -- bash -c "ros2 run map_tools cmd_vel2serial.py; exec bash"
gnome-terminal --tab --title="odo" -- bash -c "ros2 run map_tools odo; exec bash"
gnome-terminal --tab --title="serial_port" -- bash -c "ros2 run map_tools serial_port; exec bash"
gnome-terminal --tab --title="clear_cost_map" -- bash -c "ros2 run map_tools clear_cost_map; exec bash"

echo "Nav processes launched. Monitoring..."

