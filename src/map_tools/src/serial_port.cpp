#include <iostream>
#include <vector>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/navigation_msg.hpp"
#include <serial/serial.h>

#define BAUDRATE 115200

std::atomic_bool receive_thread_running;
std::atomic_bool send_thread_running;

class SerialDriverNode : public rclcpp::Node
{
public:
    SerialDriverNode(const char *nodeName) : Node(nodeName)
    {
        // 接收nav/control的数据
        sub_ = this->create_subscription<rm_interfaces::msg::NavigationMsg>(
            "/nav/control", 10, std::bind(&SerialDriverNode::callback, this, std::placeholders::_1));
        
        // 获取串口名称
        _port_name = this->declare_parameter("~port_name", "/dev/ttyUSB0");

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
                
                if (buffer.size() >= 28 && buffer[0] == 0x4A)  // 校验帧头和包的最小长度
                {
                    // 取出最后两个字节作为 CRC 校验码
                    uint16_t received_crc = ((uint16_t)buffer[25] << 8) | buffer[26];  // CRC 校验码在最后两位
                    std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + 27);  // 剩余数据部分
                    
                    // 计算 CRC 校验码
                    uint16_t calculated_crc = calculateCRC16(packet);
                    printf("CRC: %04X %04X\n",received_crc,calculated_crc);
                    //if (received_crc == calculated_crc)
                    {
                        // 如果 CRC 校验通过，处理数据包
                        processPacket(packet);
                    }
                    //else
                    {
                        //RCLCPP_ERROR(this->get_logger(), "CRC check failed. Discarding packet.");
                    }

                    buffer.clear();
                }
                else if (buffer[0] != 0x4A)
                {
                    buffer.clear();  // 清除无效数据
                }
            }
            //不能休息！！！控制那边超吊，你休息了就跟不上了
            //std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 减少CPU占用
        }
    }

    void sendThread()
    {
        send_thread_running.store(true);

        while (rclcpp::ok() && send_thread_running.load())
        {
            sendCommand();
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 每隔0.01秒发送一次
        }
    }

    void sendCommand()
    {
        try
        {
            serial_port_.write(control);
            //RCLCPP_INFO(this->get_logger(), "Command sent is OK");
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send command.");
        }
        control.clear();
    }

    void processPacket(const std::vector<uint8_t> &packet)
    {

        // 解析6个32位浮动数
        float parsed_floats[6];
        for (int i = 0; i < 6; ++i)
        {
            // 取出每个32位浮动数的4个字节
            uint8_t bytes[4] = { packet[i*4+1], packet[i*4+2], packet[i*4+3], packet[i*4+4] };
            std::memcpy(&parsed_floats[i], bytes, sizeof(float));
        }

        // 打印接收到的6个浮动数
        RCLCPP_INFO(this->get_logger(), "Received floats: ");
        for (int i = 0; i < 6; ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Float %d: %f", i, parsed_floats[i]);
        }

    }

    float i = 0;
    
    void callback(const rm_interfaces::msg::NavigationMsg::SharedPtr msg)
    {
        control.push_back((uint8_t)0xA4);  // 设置起始字节
        floatToHexBytes(msg->linear_velocity_x, control);
        floatToHexBytes(msg->linear_velocity_y, control);
        floatToHexBytes(msg->angular_velocity_z, control);

        // 添加 CRC16 校验
        uint16_t crc = calculateCRC16(control);
        control.push_back(crc & 0xFF);  // CRC16 LSB
        control.push_back((crc >> 8) & 0xFF);  // CRC16 MSB
    }

    void floatToHexBytes(float input, std::vector<uint8_t>& output)
    {
        uint8_t bytes[sizeof(float)];
        std::memcpy(bytes, &input, sizeof(float));
        for (size_t i = 0; i < sizeof(float); ++i)
        {
            output.push_back(bytes[i]);
        }
    }

    // CRC-16 Modbus 校验算法 (0xA001)
    uint16_t calculateCRC16(const std::vector<uint8_t>& data)
    {
        uint16_t crc = 0xFFFF;  // 初始值
        for (auto byte : data)
        {
            crc ^= (byte);  // 将字节移至低位
            for (int i = 0; i < 8; i++)
            {
                if (crc & 0x0001)
                {
                    crc = (crc >> 1) ^ 0xA001;  // 多项式 0xA001
                }
                else
                {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    rclcpp::Subscription<rm_interfaces::msg::NavigationMsg>::SharedPtr sub_;
    std::vector<uint8_t> control;

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
