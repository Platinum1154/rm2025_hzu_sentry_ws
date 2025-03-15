#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rm_interfaces.msg import NavigationMsg

class CmdVel2Serial(Node):
    def __init__(self):
        super().__init__('cmd_vel2serial')

        # 订阅 cmd_vel 主题
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav',  # 更改为更常见的 cmd_vel 主题
            self.cmd_vel_callback,
            10)

        # 发布合并后的 NavigationMsg
        self.publisher_ = self.create_publisher(NavigationMsg, '/nav/control', 10)

    def cmd_vel_callback(self, msg):
        # 记录接收到的 Twist 消息以便调试
        self.get_logger().info(f'接收到 Twist: linear({msg.linear.x}, {msg.linear.y}), angular({msg.angular.z})')

        # 处理接收到的 Twist 消息并转换为控制命令
        nav_msg = NavigationMsg()
        nav_msg.linear_velocity_x = msg.linear.x
        nav_msg.linear_velocity_y = msg.linear.y
        
        
        # 发布导航消息
        self.publisher_.publish(nav_msg)

        # 记录已发布的 NavigationMsg 以便调试
        self.get_logger().info(f'发布 NavigationMsg: linear({nav_msg.linear_velocity_x}, {nav_msg.linear_velocity_y})')

def main(args=None):
    rclpy.init(args=args)
    cmd_vel2serial = CmdVel2Serial()
    rclpy.spin(cmd_vel2serial)

    # 关闭
    cmd_vel2serial.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
