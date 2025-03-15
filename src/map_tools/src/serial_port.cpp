#include <iostream>
#include <vector>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>

#include "rm_interfaces/msg/odo_msg.hpp"
#include "rm_interfaces/msg/serial_receive.hpp"
#include "rm_interfaces/msg/decision.hpp"
#include "rm_interfaces/msg/navigation_msg.hpp"

#include "rm_interfaces/msg/armor_tracking.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#define BAUDRATE 115200

std::atomic_bool receive_thread_running;
std::atomic_bool send_thread_running;

class SerialDriverNode : public rclcpp::Node
{
public:
    SerialDriverNode(const char *nodeName) : Node(nodeName)
    {
        // 接收nav/control的数据
        sub_nav = this->create_subscription<rm_interfaces::msg::NavigationMsg>(
            "/nav/control", 10, std::bind(&SerialDriverNode::callback, this, std::placeholders::_1));
        sub_vision = this->create_subscription<rm_interfaces::msg::ArmorTracking>(
            "/tracker/target", 10, std::bind(&SerialDriverNode::callback_vision, this, std::placeholders::_1));

        sub_chassis_spin = this->create_subscription<std_msgs::msg::Float32>(
            "/nav/chassis_spin", 10, std::bind(&SerialDriverNode::callback_chassis_spin, this, std::placeholders::_1));

        sub_gimbal_mode = this->create_subscription<std_msgs::msg::UInt8>(
            "/nav/gimbal_mode", 10, std::bind(&SerialDriverNode::callback_gimbal_mode, this, std::placeholders::_1));

        pub_ = this->create_publisher<rm_interfaces::msg::OdoMsg>("/nav/odo", 10);
        pub_decision_ = this->create_publisher<rm_interfaces::msg::Decision>("nav/decision", 10);
        // 获取串口名称
        _port_name = this->declare_parameter("~port_name", "/dev/ttyACM0");

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
                //1+4+4+4+2*8+1+1=3
                if (buffer.size() >= 33 && buffer[0] == 0x4A && buffer[32] == 0x2b)  // 校验帧头和包的最小长度
                {
                    // for(int i =0;i<32;i++){
                    //     printf("%d:%02x\n",i,buffer[i]);
                    // }
                    // 取出最后两个字节作为 CRC 校验码
                    //uint16_t received_crc = ((uint16_t)buffer[25] << 8) | buffer[26];  // CRC 校验码在最后两位
                    std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + 32);  // 剩余数据部分
                    
                    // 计算 CRC 校验码
                    //uint16_t calculated_crc = calculateCRC16(packet);
                    //printf("CRC: %04X %04X\n",received_crc,calculated_crc);
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
                else if (buffer.size()>=33 && buffer[32] != 0x2b)
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
            second_nav++;
            second_vis++;
            if (second_nav > 1000)
            {
                global_send[0] = 0;
                global_send[1] = 0;
                global_send[2] = 5; // 巡航死机开小陀螺
            }
            if (second_vis > 1000)
            {
                global_send[3] = 0;
                global_send[4] = 0;
                global_send[5] = 0;
            }
                control.push_back((uint8_t)0xA4); // 设置起始字节
                floatToHexBytes(global_send[0], control);
                floatToHexBytes(global_send[1], control);
                floatToHexBytes(chassis_spin, control);
                floatToHexBytes(global_send[3], control);
                floatToHexBytes(global_send[4], control);
                floatToHexBytes(global_send[5], control);
                control.push_back((uint8_t)gimbal_mode); 
                control.push_back((uint8_t)0x2b); 
            // for(int i = 0; i < control.size(); i++){
            //     RCLCPP_INFO(this->get_logger(), "send %d: %x", i, control[i]);
            // }
            sendCommand();
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 发送频率200Hz
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
            //RCLCPP_ERROR(this->get_logger(), "Failed to send command.");
        }
        control.clear();
    }
    void processPacket(const std::vector<uint8_t> &packet)
    {
        // 解析3个32位浮动数
        float parsed_floats[3];
        uint16_t parsed_uint16s[8];  
        uint8_t parsed_uint8s[2];     // 用于存储 match_progress

        for (int i = 0; i < 3; ++i)
        {
            // 取出每个32位浮动数的4个字节
            uint8_t bytes[4] = { packet[i*4+1], packet[i*4+2], packet[i*4+3], packet[i*4+4] };
            std::memcpy(&parsed_floats[i], bytes, sizeof(float));
        }
        
        for (int i = 0; i < 8; ++i)  
        {
            // 取出每个16位无符号整数的2个字节
            uint8_t bytes[2] = { packet[12+i*2+1], packet[12+i*2+2] };
            std::memcpy(&parsed_uint16s[i], bytes, sizeof(uint16_t));
        }

        

        parsed_uint8s[0] = packet[29];  // 解析 match_progress
        parsed_uint8s[1] = packet[30]; 
        parsed_uint8s[2] = packet[31];
        // 打印接收到的浮动数
        // RCLCPP_INFO(this->get_logger(), "Received floats: ");
        // for (int i = 0; i < 3; ++i)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Float %d: %f", i, parsed_floats[i]);
        // }
        // for (int i = 0; i < 8; ++i)  
        // {
        //     RCLCPP_INFO(this->get_logger(), "uint16_t %d: %d", i, parsed_uint16s[i]);
        // }        
        // for (int i = 0; i < 3; ++i)  
        // {
        //     RCLCPP_INFO(this->get_logger(), "uint8_t %d: %d", i, parsed_uint8s[i]);
        // }


        // 更新决策信息
        odomsg.vx = parsed_floats[0];
        odomsg.vy = parsed_floats[1];
        odomsg.yaw = parsed_floats[2];
        pub_->publish(odomsg);
        
        decision.self_sentry_hp = parsed_uint16s[0];         // 我方哨兵血量
        decision.self_hero_hp = parsed_uint16s[1];           // 我方英雄血量
        decision.self_infantry_hp = parsed_uint16s[2];       // 我方步兵血量
        decision.enemy_sentry_hp = parsed_uint16s[3];        // 敌方哨兵血量
        decision.enemy_hero_hp = parsed_uint16s[4];          // 敌方英雄血量
        decision.enemy_infantry_hp = parsed_uint16s[5];      // 敌方步兵血量
        decision.remain_time = parsed_uint16s[6];            // 剩余时间
        decision.remain_bullet = parsed_uint16s[7];          // 剩余子弹
        decision.match_progress = parsed_uint8s[0];                // 比赛状况
        decision.occupation = parsed_uint8s[1];              // 事件
        decision.color = parsed_uint8s[2];              // 事件
        pub_decision_->publish(decision);
    }
    float i = 0;
    
    void callback(const rm_interfaces::msg::NavigationMsg::SharedPtr msg)
    {
        second_nav=0;
        global_send[0] = msg->linear_velocity_x;
        global_send[1] = msg->linear_velocity_y;
        global_send[2] =1; // 小陀螺（0-100）
        gimbal_mode = 1;
        // if(control.size()==13){
        //     floatToHexBytes(0, control);
        //     floatToHexBytes(0, control);
        //     floatToHexBytes(0, control);
        //     control.push_back((uint8_t)0x2b); 
        // }
        // if(control.size()<1){
        //     control.push_back((uint8_t)0xA4);  // 设置起始字节
        //     floatToHexBytes(msg->linear_velocity_x, control);
        //     floatToHexBytes(msg->linear_velocity_y, control);
        //     floatToHexBytes(55.55, control);
        // }
    }

    void callback_vision(const rm_interfaces::msg::ArmorTracking::SharedPtr msg)
    {
        second_vis=0;
        global_send[3] = msg->yaw;
        global_send[4] = msg->pitch;
        global_send[5] = msg->deep;
        // if(control.size()<1){
        //     control.push_back((uint8_t)0xA4);  // 设置起始字节
        //     floatToHexBytes(0, control);
        //     floatToHexBytes(0, control);
        //     floatToHexBytes(0, control);
        //     floatToHexBytes(msg->yaw, control);
        //     floatToHexBytes(msg->pitch, control);
        //     floatToHexBytes(msg->deep, control);
        //     control.push_back((uint8_t)0x2b); 
        // }
        // else if(control.size()==13){
        //     floatToHexBytes(msg->yaw, control);
        //     floatToHexBytes(msg->pitch,control);
        //     floatToHexBytes(msg->deep,control);
        //     control.push_back((uint8_t)0x2b); 
        // }
    }
    void callback_chassis_spin(const std_msgs::msg::Float32::SharedPtr msg)
    {
        chassis_spin = msg->data;
    }

    void callback_gimbal_mode(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        gimbal_mode = msg->data;
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

    rclcpp::Subscription<rm_interfaces::msg::NavigationMsg>::SharedPtr sub_nav;    rclcpp::Subscription<rm_interfaces::msg::ArmorTracking>::SharedPtr sub_vision;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_chassis_spin;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_gimbal_mode;
    std::vector<uint8_t> control;
    rclcpp::Publisher<rm_interfaces::msg::OdoMsg>::SharedPtr pub_;
    rclcpp::Publisher<rm_interfaces::msg::Decision>::SharedPtr pub_decision_;
    rm_interfaces::msg::OdoMsg odomsg;
    rm_interfaces::msg::Decision decision;
    serial::Serial serial_port_;
    std::string _port_name;
    volatile long second_nav = 0, second_vis = 0;
    std::thread receive_thread_;
    std::thread send_thread_;
    float global_send[6];
    float chassis_spin;
    uint8_t gimbal_mode;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto serial_node = std::make_shared<SerialDriverNode>("serial_driver_node");

    rclcpp::spin(serial_node);

    rclcpp::shutdown();
    return 0;
}
