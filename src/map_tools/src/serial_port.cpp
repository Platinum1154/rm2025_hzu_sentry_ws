#include <iostream>
#include <chrono>
#include <cmath>
#include <serial/serial.h>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "rclcpp/rclcpp.hpp"

#define BAUDRATE 115200

// std::atomic_bool 是 C++ 标准库中的原子布尔类型。它是一种特殊的数据类型，用于在多线程环境中进行原子操作。
// 原子操作是指不可中断的操作，要么完全执行，要么完全不执行。std::atomic_bool 类型提供了原子性的读取和写入操作，以确保多个线程对该变量的访问不会导致竞争条件或数据不一致的问题。
std::atomic_bool imu_thread_running;
std::atomic_bool imu_data_ready;
std::vector<uint8_t> buff;
std::vector<int16_t> acceleration(4, 0);
std::vector<int16_t> angularVelocity(4, 0);
std::vector<int16_t> magnetometer(4, 0);
std::vector<int16_t> angle_degree(4, 0);

class IMUDriverNode : public rclcpp::Node
{
public:
    IMUDriverNode(const char *nodeName) : Node(nodeName)
    {
        // 获取串口
        _port_name = this->declare_parameter("~port_name", "/dev/ttyUSB0");
        // 发布IMU数据
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_raw", 10);
        // IMU驱动线程
        imu_thread_ = std::thread(&IMUDriverNode::imuThread, this, _port_name);
    }

    void joinIMUThread()
    {
        imu_thread_.join();
    }

private:
    // IMU驱动线程
    void imuThread(const std::string &port_name)
    {
        // 1 创建串口对象
        serial::Serial imu_serial;
        // 串口初始化
        try
        {
            imu_serial.setPort(port_name);
            imu_serial.setBaudrate(BAUDRATE);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(500);
            imu_serial.setTimeout(timeout);
            imu_serial.open();
            RCLCPP_INFO(this->get_logger(), "\033[32mSerial port opened successfully...\033[0m");
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the IMU serial port.");
            return;
        }

        // Clear the buffer
        imu_serial.flush();

        // 2 循环读取串口数据
        while (rclcpp::ok() && imu_thread_running.load())
        {
            if (imu_serial.available())
            {
                uint8_t data;
                imu_serial.read(&data, 1);
                // 一位一位地添加到队列
                buff.push_back(data);

                // 当大于11个数据位时解析数据，四种数据的第一位均为0x55
                if (buff.size() >= 11 && buff[0] == 0x55)
                {
                    // 获取11位数据
                    std::vector<uint8_t> data_buff(buff.begin(), buff.begin() + 11);
                    // 获取中间数字位
                    std::vector<uint8_t> data(buff.begin() + 2, buff.begin() + 10);
                    // 获取完一帧的标志位
                    bool angle_flag = false;

                    // 加速度数据
                    if (data_buff[1] == 0x51)
                    {
                        // 检查检验和
                        if (checkSum(data_buff))
                            // 获取16位数据
                            acceleration = hexToShort(data);
                        else
                            RCLCPP_WARN(this->get_logger(), "0x51 Check failure.");
                    }
                    // 下面的都类似
                    else if (data_buff[1] == 0x52)
                    {
                        if (checkSum(data_buff))
                            angularVelocity = hexToShort(data);
                        else
                            RCLCPP_WARN(this->get_logger(), "0x52 Check failure.");
                    }
                    else if (data_buff[1] == 0x53)
                    {
                        if (checkSum(data_buff))
                        {
                            angle_degree = hexToShort(data);
                            angle_flag = true;
                        }
                        else
                            RCLCPP_WARN(this->get_logger(), "0x53 Check failure.");
                    }
                    // 实际上没用到
                    else if (data_buff[1] == 0x54)
                    {
                        if (checkSum(data_buff))
                            magnetometer = hexToShort(data);
                        else
                            RCLCPP_WARN(this->get_logger(), "0x54 Check failure.");
                    }

                    buff.clear();

                    // 已经获取角度数据（顺序第三个），可以发布了
                    if (angle_flag)
                    {
                        // 可以输出数据的标志位
                        imu_data_ready.store(true);
                    }
                }
                else if (buff[0] != 0x55)
                {
                    buff.clear();
                }
            }

            if (imu_data_ready.load())
            {
                sensor_msgs::msg::Imu msg;
                msg.header.stamp = this->now();
                msg.header.frame_id = "base_link";

                // 将16数据转化位double
                // Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
                // 加速度X=((AxH<<8)|AxL)/32768*16g(g为重力加速度，可取9.8m/s2)
                msg.linear_acceleration.x = static_cast<double>(acceleration[0]) / 32768.0 * 16 * 9.8;
                msg.linear_acceleration.y = static_cast<double>(acceleration[1]) / 32768.0 * 16 * 9.8;
                msg.linear_acceleration.z = static_cast<double>(acceleration[2]) / 32768.0 * 16 * 9.8;

                // 角速度X=((WxH<<8)|WxL)/32768*2000 °/s
                // 转化为弧度X=((WxH<<8)|WxL)/32768*2000*PI/180 rad/s
                msg.angular_velocity.x = static_cast<double>(angularVelocity[0]) / 32768.0 * 2000 * M_PI / 180;
                msg.angular_velocity.y = static_cast<double>(angularVelocity[1]) / 32768.0 * 2000 * M_PI / 180;
                msg.angular_velocity.z = static_cast<double>(angularVelocity[2]) / 32768.0 * 2000 * M_PI / 180;

                // 滚转角X=((RollH<<8)|RollL)/32768*180 (°)
                // 转化为弧度X=((RollH<<8)|RollL)/32768*180*PI/180=((RollH<<8)|RollL)/32768*PI (rad)
                double roll = static_cast<double>(angle_degree[0]) / 32768.0 * M_PI;
                double pitch = static_cast<double>(angle_degree[1]) / 32768.0 * M_PI;
                double yaw = static_cast<double>(angle_degree[2]) / 32768.0 * M_PI;
                tf2::Quaternion quat;
                quat.setRPY(roll, pitch, yaw);
                // Assign quaternion to msg.orientation
                msg.orientation.x = quat.getX();
                msg.orientation.y = quat.getY();
                msg.orientation.z = quat.getZ();
                msg.orientation.w = quat.getW();

                publisher_->publish(msg);
                imu_data_ready.store(false);
            }
        }
        imu_serial.close();
    }

    // 检验和：四个数据的检验和计算方式是一样的，所以统一写了
    bool checkSum(const std::vector<uint8_t> &data_buff)
    {
        uint8_t sum = 0;
        for (size_t i = 0; i < data_buff.size() - 1; i++)
        {
            sum += data_buff[i];
        }

        // 计算结果是否与输出一致
        return sum == data_buff[data_buff.size() - 1];
    }

    // 解析数据
    std::vector<int16_t> hexToShort(const std::vector<uint8_t> &hex_data)
    {
        std::vector<int16_t> short_data(4, 0);
        for (size_t i = 0; i < hex_data.size(); i += 2)
        {
            // 将两个连续的字节（低位hex_data[i] 和 高位hex_data[i+1]）组合为一个 int16_t 类型的数值：千万注意高低顺序，仔细看通讯协议
            // 高位hex_data[i+1]需要先强制转换为一个有符号的short类型的数据以后再移位
            short high = static_cast<int16_t>(hex_data[i + 1]);
            // 左移运算符 << 将 high 的二进制表示向左移动 8 位。这样做是因为 int16_t 类型占用 2 个字节，而我们希望将 high 的数据放置在最高的 8 位上。
            // |: 按位或运算符 | 将经过左移的 high 数据和 hex_data[i] 数据进行按位或操作，将它们组合为一个 16 位的数值
            short_data[i / 2] = static_cast<int16_t>((high << 8) | hex_data[i]);
        }

        return short_data;
    }

    // 与上面那个函数是等价的，直接采用C++函数解析
    std::vector<int16_t> hexToShort1(const std::vector<uint8_t> &hex_data)
    {
        char rawData[] = {hex_data[0], hex_data[1], hex_data[2], hex_data[3], hex_data[4], hex_data[5], hex_data[6], hex_data[7]};
        // 创建一个存储解析后整数的数组
        short result[4];
        // 使用memcpy将字节数组解析为整数数组
        memcpy(result, rawData, sizeof(result));

        std::vector<int16_t> short_data = {result[0], result[1], result[2], result[3]};
        return short_data;
    }

    std::string _port_name;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
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
