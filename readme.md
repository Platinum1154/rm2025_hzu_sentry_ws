# rm2025_hzu_sentry_ws

面向 RoboMaster 哨兵机器人的 ROS2 自主导航工作空间，集成双激光雷达、SLAM 建图、Nav2 自主导航、激光里程计、串口通信和比赛状态决策逻辑。

本项目主要用于哨兵机器人在场地中的自主巡航、定点导航、低血量补给点回退、局部避障和底盘控制指令下发。

## 项目简介

`rm2025_hzu_sentry_ws` 是一个基于 ROS2 Humble 的机器人导航系统，适用于 RoboMaster 场景下的哨兵机器人。系统通过前后双 YDLIDAR X3 激光雷达获取环境信息，将双雷达数据融合为统一的激光扫描数据，并结合 rf2o 激光里程计、Gmapping 建图、Nav2 导航栈和自定义串口协议，实现机器人在比赛环境中的自主移动与决策。

系统主要能力包括：

- 双激光雷达数据融合
- SLAM 建图与地图保存
- 基于已有地图的 Nav2 自主导航
- rf2o 激光里程计
- 定点导航与巡航逻辑
- 根据比赛状态和机器人血量执行导航策略
- ROS2 `cmd_vel` 到自定义串口控制协议的转换
- 局部代价地图清除
- RViz 可视化与 TF 调试

## 运行环境

推荐环境：

- Ubuntu 22.04
- ROS2 Humble
- Nav2
- Gmapping
- YDLIDAR SDK / ydlidar_ros2_driver
- Python 3
- colcon
- gnome-terminal

建议先安装常用 ROS2 依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rviz2 \
  ros-humble-tf2-tools \
  ros-humble-rqt-tf-tree \
  ros-humble-rqt-reconfigure \
  python3-colcon-common-extensions
```

> 注意：实际依赖可能会随本地环境、雷达 SDK、串口设备和机器人底盘控制板配置而变化。

## 项目结构

```text
rm2025_hzu_sentry_ws/
├── src/
│   ├── bot_navigation2/                 # Nav2 导航启动与参数配置
│   ├── map_tools/                       # 地图、导航目标、串口封装、雷达融合等工具
│   ├── openslam_gmapping/               # Gmapping 底层实现
│   ├── slam_gmapping/                   # ROS2 Gmapping 封装与启动文件
│   ├── rf2o_laser_odometry-ros2/        # rf2o 激光里程计
│   ├── rm_interfaces/                   # 自定义 ROS2 消息接口
│   ├── rm_serial_python/                # Python 串口通信节点
│   └── ydlidar_ros2_driver-master/      # YDLIDAR 雷达驱动
├── 地图备份/                            # 地图备份
├── slam_map.yaml                        # 当前导航地图配置
├── slam_map.pgm                         # 当前导航地图图像
├── begin.sh                             # 一键启动脚本
├── begin2.sh                            # 备用启动脚本
├── start.sh                             # 启动脚本
├── stop.sh                              # 停止脚本
└── readme.md                            # 原始运行说明
```

## 核心模块说明

### 1. bot_navigation2

主要负责 Nav2 导航栈启动和参数配置，包括：

- 地图导航
- 代价地图配置
- 全局路径规划
- 局部路径规划
- RViz 可视化配置
- 测试用整合启动文件

常用启动文件：

```bash
ros2 launch bot_navigation2 test_all.launch.py
ros2 launch bot_navigation2 navigation2.launch.py use_sim_time:=False slam:=False map:=/home/dt46/rm2025_hzu_sentry_ws/slam_map.yaml
```

### 2. map_tools

项目中的导航工具包，负责连接导航层和底盘通信层，包含：

- 双雷达数据融合
- 静态 TF 发布
- 定点导航
- `cmd_vel` 转串口数据
- 串口通信
- 里程计节点
- 清除局部代价地图

常用节点：

```bash
ros2 run map_tools nav2_to_goal
ros2 run map_tools cmd_vel2serial.py
ros2 run map_tools serial_port
ros2 run map_tools odo
ros2 run map_tools clear_cost_map
```

### 3. slam_gmapping / openslam_gmapping

用于 SLAM 建图。系统可以通过激光雷达数据构建占据栅格地图，并保存为 Nav2 可加载的 `yaml + pgm` 地图文件。

启动建图：

```bash
ros2 launch slam_gmapping test_gmapping.launch.py
```

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -t map -f slam_map
```

### 4. rf2o_laser_odometry-ros2

用于通过连续激光扫描匹配计算机器人位姿变化，提供激光里程计信息。

启动：

```bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
```

### 5. ydlidar_ros2_driver-master

YDLIDAR X3 雷达驱动。项目使用两路雷达数据，分别作为前后雷达输入。

启动一号雷达：

```bash
ros2 launch ydlidar_ros2_driver x3_ydlidar_launch.py
```

启动二号雷达：

```bash
ros2 launch ydlidar_ros2_driver x3_ydlidar_launch_2.py
```

### 6. rm_serial_python

Python 版本串口通信包，用于和下位机或裁判系统相关模块通信。可用于调试或替代部分 C++ 串口节点。

启动：

```bash
ros2 run rm_serial_python rm_serial_node
```

### 7. rm_interfaces

自定义 ROS2 消息接口包，用于系统内部传递导航、决策和串口相关数据。

## 编译

进入工作空间根目录：

```bash
cd ~/rm2025_hzu_sentry_ws
```

加载 ROS2 环境：

```bash
source /opt/ros/humble/setup.bash
```

编译全部包：

```bash
colcon build
```

只编译指定包，例如 `map_tools`：

```bash
colcon build --packages-select map_tools
```

编译完成后加载工作空间环境：

```bash
source install/setup.bash
```

建议将环境加载命令加入 `~/.bashrc`：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/rm2025_hzu_sentry_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 快速启动

项目提供 `begin.sh` 用于启动主要导航流程。

```bash
cd ~/rm2025_hzu_sentry_ws
chmod +x begin.sh
./begin.sh
```

`begin.sh` 会依次启动：

- Nav2 地图导航
- 雷达与 TF 相关整合启动
- 定点导航节点
- `cmd_vel` 串口数据封装节点
- 里程计节点
- 串口通信节点
- 局部代价地图清除节点

如果你的工作空间路径不是：

```text
/home/dt46/rm2025_hzu_sentry_ws
```

需要修改 `begin.sh` 中的路径，例如：

```bash
source /home/dt46/rm2025_hzu_sentry_ws/install/setup.bash
```

以及地图路径：

```bash
map:=/home/dt46/rm2025_hzu_sentry_ws/slam_map.yaml
```

改为你本机实际路径。

## 建图流程

### 1. 启动雷达与相关节点

可以使用整合启动：

```bash
ros2 launch bot_navigation2 test_all.launch.py
```

也可以分别启动：

```bash
ros2 launch ydlidar_ros2_driver x3_ydlidar_launch.py
ros2 launch ydlidar_ros2_driver x3_ydlidar_launch_2.py
ros2 launch map_tools merged_scan.launch.py
ros2 launch map_tools tf_static_launch.py
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
```

### 2. 启动 Gmapping 建图

```bash
ros2 launch slam_gmapping test_gmapping.launch.py
```

### 3. 打开 RViz 查看建图效果

```bash
rviz2
```

### 4. 保存地图

```bash
ros2 run nav2_map_server map_saver_cli -t map -f slam_map
```

保存后会生成：

```text
slam_map.yaml
slam_map.pgm
```

## 导航流程

### 1. 启动里程计

```bash
ros2 run map_tools odo
```

### 2. 启动 Nav2 导航

```bash
ros2 launch bot_navigation2 navigation2.launch.py use_sim_time:=False slam:=False map:=/home/dt46/rm2025_hzu_sentry_ws/slam_map.yaml
```

### 3. 启动定点导航逻辑

```bash
ros2 run map_tools nav2_to_goal
```

### 4. 启动速度指令转换节点

```bash
ros2 run map_tools cmd_vel2serial.py
```

### 5. 启动串口通信

真串口：

```bash
ros2 run map_tools serial_port
```

Python 串口节点：

```bash
ros2 run rm_serial_python rm_serial_node
```

### 6. 清除局部代价地图

单次清除：

```bash
ros2 run map_tools clear_cost_map
```

## 常用调试命令

查看 TF 树：

```bash
ros2 run rqt_tf_tree rqt_tf_tree --force-discover
```

打开 RViz：

```bash
rviz2
```

动态调参：

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

手动加载地图服务器：

```bash
ros2 run nav2_map_server map_server --ros-args --param yaml_filename:=slam_map.yaml
```

配置地图服务器生命周期：

```bash
ros2 lifecycle set /map_server configure
```

激活地图服务器：

```bash
ros2 lifecycle set /map_server activate
```

查看话题：

```bash
ros2 topic list
```

查看 TF：

```bash
ros2 run tf2_tools view_frames
```

查看节点：

```bash
ros2 node list
```

## 一键启动脚本示例

`begin.sh` 的主要逻辑如下：

```bash
#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/dt46/rm2025_hzu_sentry_ws/install/setup.bash

gnome-terminal --tab --title="navigation2_copy.launch.py" -- bash -c "ros2 launch bot_navigation2 navigation2_copy.launch.py use_sim_time:=False slam:=False map:=/home/dt46/rm2025_hzu_sentry_ws/slam_map.yaml; exec bash"

sleep 5

gnome-terminal --tab --title="test_all.launch.py" -- bash -c "ros2 launch bot_navigation2 test_all.launch.py; exec bash"
gnome-terminal --tab --title="nav2_to_goal" -- bash -c "ros2 run map_tools nav2_to_goal; exec bash"
gnome-terminal --tab --title="cmd_vel2serial.py" -- bash -c "ros2 run map_tools cmd_vel2serial.py; exec bash"
gnome-terminal --tab --title="odo" -- bash -c "ros2 run map_tools odo; exec bash"
gnome-terminal --tab --title="serial_port" -- bash -c "ros2 run map_tools serial_port; exec bash"
gnome-terminal --tab --title="clear_cost_map" -- bash -c "ros2 run map_tools clear_cost_map; exec bash"
```

使用前请根据实际路径修改工作空间路径和地图路径。

## 系统运行逻辑

系统整体数据流大致如下：

```text
YDLIDAR 前雷达       YDLIDAR 后雷达
      │                    │
      └──────┬─────────────┘
             │
      双雷达数据融合
             │
      /scan 或合并 scan
             │
      ┌──────┴─────────┐
      │                │
  Gmapping 建图     rf2o 激光里程计
      │                │
      └──────┬─────────┘
             │
           Nav2
             │
        /cmd_vel
             │
     cmd_vel2serial.py
             │
       serial_port
             │
          下位机
```

比赛决策相关逻辑由 `nav2_to_goal` 等节点实现，可根据机器人状态、血量、比赛阶段等信息选择巡航点或补给点。

## 注意事项

1. 启动前确认雷达串口权限和设备名是否正确。
2. 如果使用真串口，请确认下位机通信协议与节点中的数据格式一致。
3. `begin.sh` 中包含固定绝对路径，换机器后需要手动修改。
4. 地图文件路径需要和 Nav2 启动参数一致。
5. 如果导航过程中局部代价地图出现异常障碍物，可运行 `clear_cost_map`。
6. 建图和导航不要同时混用错误地图坐标系，启动前确认 TF 树是否正常。
7. 首次运行前建议单独启动各模块，确认雷达、TF、里程计、Nav2 和串口均正常后再使用一键脚本。

## License

本项目使用 MIT License。具体内容见仓库中的 `LICENSE` 文件。