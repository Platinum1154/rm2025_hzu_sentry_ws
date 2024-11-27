import time
import serial
import threading
import struct
import zlib

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header  # 字符串消息类型和头部消息类型
from rm_interfaces.msg import ArmorTracking, SerialReceive, NavigationMsg  # 导入自定义消息类型

class RMSerialDriver(Node):
    def __init__(self, name):
        super().__init__(name)

        self.get_logger().info("启动 RMSerialDriver!")

        # 获取参数
        self.get_params()

        # 创建订阅者
        self.sub_tracker = self.create_subscription(
            ArmorTracking, "/tracker/target", self.send_data, 10
        )

        self.sub_nav = self.create_subscription(
            NavigationMsg, "/nav/control", self.send_data_nav, 10
        )

        # 创建发布者
        self.pub_uart_receive = self.create_publisher(NavigationMsg, "/uart/receive", 10)
        
        # 创建变量
        self.tracking_color = 0
        
        # 初始化串口
        try:
            self.serial_port = serial.Serial(
                port=self.device_name,
                baudrate=self.baud_rate,
                timeout=1,
                write_timeout=1,
            )
            if self.serial_port.is_open:
                self.get_logger().info("创建串口 successfully.")
                self.receive_thread = threading.Thread(target=self.receive_data)
                self.receive_thread.start()

        except serial.SerialException as e:
            self.get_logger().error(f"创建串口时出错: {self.device_name} - {str(e)}")
            raise e

    def get_params(self):
        """获取并设置串口相关的参数"""
        self.device_name  = self.declare_parameter("device_name", "/dev/ttyUSB0").value
        self.baud_rate    = self.declare_parameter("baud_rate", 115200).value
        self.flow_control = self.declare_parameter("flow_control", "none").value
        self.parity       = self.declare_parameter("parity", "none").value
        self.stop_bits    = self.declare_parameter("stop_bits", "1").value
        
    def receive_data(self):
        """接收串口数据并处理"""
        while rclpy.ok():
            time.sleep(0.5)  # 1秒接受一次串口数据
            
            try:
                serial_receive_msg = SerialReceive()  # 创建并设置消息
                serial_receive_msg.header = Header()  # 创建并设置Header
                serial_receive_msg.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
                serial_receive_msg.header.frame_id = 'serial_receive_frame'  # 可根据需要设置frame_id
                                
                # 读取数据头部
                header = self.serial_port.read(1)
                if header and header[0] == 0xA5:  # 视觉数据
                    data = self.serial_port.read(16)  # 读取16字节的数据

                    if len(data) == 16:  
                        try: 
                            packet = struct.unpack("<B?fffH", header + data)  # 注意这里的格式字符串

                            self.get_logger().info(f"解包收到的数据: {packet}")
                        except serial.SerialException as e:
                            self.get_logger().error(f"接收数据时出错: {str(e)}")
                            self.reopen_port()
                        serial_receive_msg.data = packet  # 给ros消息装入数据

                        # 更新目标颜色参数
                        if packet[1] != self.tracking_color:
                            self.tracking_color = packet[1]
                            # 更新颜色
                            serial_receive_msg.tracking_color = self.tracking_color
                            
                            # 发送ROS消息
                            self.pub_uart_receive.publish(serial_receive_msg)
                    else:
                        self.get_logger().warn("Received data length mismatch1")

                elif header and header[0] == 0x4A:  # 导航数据
                    data = self.serial_port.read(24)  # 读取22字节的数据

                    if len(data) == 24:
                        try:
                            # 解包数据
                            # packet = data
                            # print(packet.hex())

                            packet = struct.unpack("<Bfff3f", header + data)  # 注意这里的格式字符串

                            # # 验证校验和
                            # received_checksum = packet[7]
                            # calculated_checksum = self.calculate_checksum(data[:20])
                            # if received_checksum != calculated_checksum:
                            #     self.get_logger().warn("Checksum mismatch, data may be corrupted")
                            #     continue
                            self.get_logger().warn("i got u!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            self.get_logger().info(f"解包收到的数据: {packet}")

                            # 将解包后的数据放入ROS消息中
                            nav_msg = NavigationMsg()
                            nav_msg.header = serial_receive_msg.header
                            nav_msg.vx = packet[1]
                            nav_msg.vy = packet[2]
                            nav_msg.vz = packet[3]
                            nav_msg.gyro_x = packet[4]
                            nav_msg.gyro_y = packet[5]
                            nav_msg.gyro_z = packet[6]

                            # 发布导航消息
                            self.pub_uart_receive.publish(nav_msg)
                        except serial.SerialException as e:
                            self.get_logger().error(f"接收数据时出错: {str(e)}")
                            self.reopen_port()
                    else:
                        self.get_logger().warn("Received data length mismatch2")

            except Exception as e:
                self.get_logger().error(f"Unexpected error: {str(e)}")

    def calculate_checksum(self, data):
        """计算校验和"""
        checksum = 0
        for byte in data:
            checksum += byte
        return checksum & 0xFFFF

    def reopen_port(self):
        """重新打开串口"""
        if self.serial_port.is_open:
            self.serial_port.close()
        try:
            self.serial_port.open()
        except serial.SerialException as e:
            self.get_logger().error(f"Reopening serial port failed: {str(e)}")

    def send_data(self, msg):
        # 处理发送数据的逻辑
        pass

    def send_data_nav(self, msg):
        # 处理发送导航数据的逻辑
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RMSerialDriver('rm_serial_driver')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()