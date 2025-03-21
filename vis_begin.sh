echo "Sourcing ROS 2 environment..."
source /opt/ros/humble/setup.bash
source /home/morefine/ros_ws/install/setup.bash
gnome-terminal --tab --title="rcv" -- bash -c "ros2 launch rm_vision_bringup opencv_nav.launch.py; exec bash"
echo "Vision processes launched. Monitoring..."
