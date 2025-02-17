cmake_minimum_required(VERSION 3.5)
project(map_tools)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(merger_node src/laser_scan_merger.cpp)
add_executable(save_point_cloud src/save_point_cloud.cpp)
add_executable(serial_port src/serial_port.cpp)
add_executable(clear_cost_map src/clear_cost_map.cpp)
add_executable(odo src/odo.cpp)
add_executable(nav2_to_goal src/nav2_to_goal.cpp)

# add_executable(cmd_vel2serial src/cmd_vel2serial.cpp)



ament_target_dependencies(merger_node
  rclcpp 
  sensor_msgs 
  laser_geometry 
  pcl_ros 
  PCL
  rm_interfaces
  std_msgs
)

ament_target_dependencies(save_point_cloud
  rclcpp 
  sensor_msgs 
  laser_geometry 
  pcl_ros 
  PCL
  rm_interfaces
  std_msgs
)

ament_target_dependencies(serial_port 
  rclcpp 
  sensor_msgs 
  laser_geometry 
  pcl_ros 
  PCL
  rm_interfaces
  std_msgs
  serial
)

ament_target_dependencies(clear_cost_map 
  rclcpp 
  sensor_msgs 
  laser_geometry 
  pcl_ros 
  PCL
  rm_interfaces
  std_msgs
  std_srvs
  nav2_msgs
)

ament_target_dependencies(odo 
  rclcpp 
  sensor_msgs 
  laser_geometry 
  pcl_ros 
  PCL
  rm_interfaces
  std_msgs
  std_srvs
  nav2_msgs
)

ament_target_dependencies(nav2_to_goal 
  rclcpp 
  sensor_msgs 
  laser_geometry 
  pcl_ros 
  PCL
  rm_interfaces
  std_msgs
  std_srvs
  nav2_msgs
)
# ament_target_dependencies(cmd_vel2serial
#   rclcpp 
#   sensor_msgs 
#   laser_geometry 
#   pcl_ros 
#   PCL
#   rm_interfaces
#   std_msgs
# )




install(TARGETS
  serial_port
  merger_node
  save_point_cloud
  clear_cost_map
  odo
  nav2_to_goal
  # cmd_vel2serial
  DESTINATION lib/${PROJECT_NAME})
  



# 安装 launch 目录
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# 安装 config 目录中的配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)



ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/cmd_vel2serial.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
