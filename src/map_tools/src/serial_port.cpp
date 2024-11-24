#include <iostream>
#include <chrono>
#include <thread>
#include <serial/serial.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#define BAUDRATE 9600

std::atomic_bool receive_thread_running;
std::atomic_bool send_thread_running;

class SerialDriverNode : public rclcpp::Node
{
public:
    SerialDriverNode(const char *nodeName) : Node(nodeName)
    {
        // 获取串口名称
        _port_name = this->declare_parameter("~port_name", "/dev/ttyUSB2");

        // 初始化串口
        try
        {
            serial_port_.setPort(_port_name);
            serial_port_.setBaudrate(BAUDRATE);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(500);
            serial_port_.setTimeout(timeout);
            serial_port_.open();
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully...");
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the serial port.");
            return;
        }

        serial_port_.flush();

        // 启动接收线程
        receive_thread_ = std::thread(&SerialDriverNode::receiveThread, this);

        // 启动发送线程
        send_thread_ = std::thread(&SerialDriverNode::sendThread, this);
    }

    ~SerialDriverNode()
    {
        // 停止线程
        receive_thread_running.store(false);
        send_thread_running.store(false);

        if (receive_thread_.joinable())
            receive_thread_.join();
        if (send_thread_.joinable())
            send_thread_.join();

        if (serial_port_.isOpen())
            serial_port_.close();
    }

private:
    void receiveThread()
    {
        receive_thread_running.store(true);
        std::vector<uint8_t> buffer;

        while (rclcpp::ok() && receive_thread_running.load())
        {
            if (serial_port_.available())
            {
                uint8_t data;
                serial_port_.read(&data, 1);
                buffer.push_back(data);

                if (buffer.size() >= 4 && buffer[0] == 0x55)
                {
                    std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + 4);
                    processPacket(packet);

                    buffer.clear();
                }
                else if (buffer[0] != 0x55)
                {
                    buffer.clear();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 减少CPU占用
        }
    }

    void sendThread()
    {
        send_thread_running.store(true);

        while (rclcpp::ok() && send_thread_running.load())
        {
            sendCommand();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 每隔1秒发送一次
        }
    }

    void sendCommand()
    {
        // 定义固定内容的命令
        std::vector<uint8_t> command = {0xA1, 0xB2, 0xC3, 0xD4}; // 替换为你的实际命令
        try
        {
            serial_port_.write(command);
            RCLCPP_INFO(this->get_logger(), "Command sent: A1 B2 C3 D4");
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send command.");
        }
    }

    // 处理数据包
    void processPacket(const std::vector<uint8_t> &packet)
    {
        RCLCPP_INFO(this->get_logger(), "Packet received: ");
        for (const auto &byte : packet)
        {
            printf("%02X ", byte);
        }
        printf("\n");
    }

    serial::Serial serial_port_;
    std::string _port_name;

    std::thread receive_thread_;
    std::thread send_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto serial_node = std::make_shared<SerialDriverNode>("serial_driver_node");

    rclcpp::spin(serial_node);

    rclcpp::shutdown();
    return 0;
}
