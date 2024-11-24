#include <iostream>
#include <chrono>
#include <thread>
#include <serial/serial.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#define BAUDRATE 9600

std::atomic_bool imu_thread_running;

class IMUDriverNode : public rclcpp::Node
{
public:
    IMUDriverNode(const char *nodeName) : Node(nodeName)
    {
        // 获取串口
        _port_name = this->declare_parameter("~port_name", "/dev/ttyUSB2");

        // 创建IMU驱动线程
        imu_thread_ = std::thread(&IMUDriverNode::imuThread, this, _port_name);
    }

    void joinIMUThread()
    {
        imu_thread_.join();
    }

private:
    void imuThread(const std::string &port_name)
    {
        serial::Serial imu_serial;

        try
        {
            imu_serial.setPort(port_name);
            imu_serial.setBaudrate(BAUDRATE);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(500);
            imu_serial.setTimeout(timeout);
            imu_serial.open();
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully...");
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the IMU serial port.");
            return;
        }

        imu_serial.flush();

        std::vector<uint8_t> buff;

        // 循环读取数据
        while (rclcpp::ok() && imu_thread_running.load())
        {
            // 定时发送固定内容
            sendCommand(imu_serial);

            if (imu_serial.available())
            {
                uint8_t data;
                imu_serial.read(&data, 1);
                buff.push_back(data);

                if (buff.size() >= 4 && buff[0] == 0x55)
                {
                    std::vector<uint8_t> data_buff(buff.begin(), buff.begin() + 4);
                    processPacket(data_buff);
                    //-*-*-先去掉校验和
                    // if (checkSum(data_buff))
                    // {
                    //     processPacket(data_buff);
                    // }
                    // else
                    // {
                    //     RCLCPP_WARN(this->get_logger(), "Checksum failed.");
                    // }

                    buff.clear();
                }
                else if (buff[0] != 0x55)
                {
                    buff.clear();
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        imu_serial.close();
    }

    // 发送固定内容
    void sendCommand(serial::Serial &imu_serial)
    {
        // 定义固定内容的命令
        std::vector<uint8_t> command = {0xA1, 0xB2, 0xC3, 0xD4}; // 替换为你的实际命令
        try
        {
            imu_serial.write(command);
            RCLCPP_INFO(this->get_logger(), "Command sent: A1 B2 C3 D4");
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send command.");
        }
    }

    // 校验和函数
    bool checkSum(const std::vector<uint8_t> &data_buff)
    {
        uint8_t sum = 0;
        for (size_t i = 0; i < data_buff.size() - 1; i++)
        {
            sum += data_buff[i];
        }
        return sum == data_buff.back();
    }

    // 处理数据包
    void processPacket(const std::vector<uint8_t> &data_buff)
    {
        // 示例：打印数据包内容
        RCLCPP_INFO(this->get_logger(), "Packet received: ");
        for (const auto &byte : data_buff)
        {
            printf("%02X ", byte);
        }
        printf("\n");

        // 在这里可以根据需要解析和处理数据包
    }

    std::string _port_name;
    std::thread imu_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto imu_node = std::make_shared<IMUDriverNode>("imu_driver_node");
    imu_thread_running.store(true);

    rclcpp::spin(imu_node);
    imu_thread_running.store(false);

    imu_node->joinIMUThread();
    rclcpp::shutdown();
    return 0;
}
