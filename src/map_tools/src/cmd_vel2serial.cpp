#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rm_interfaces/msg/NavigationMsg.hpp"
// #include "~/rm2025_hzu_sentry_ws/src/rm_interfaces/msg/NavigationMsg.msg"
class cmd_vel2serial : public rclcpp::Node {
public:
    cmd_vel2serial() : Node("cmd_vel2serial") {
        // 订阅cmd_vel话题
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, std::bind(&cmd_vel2serial::Callback, this, std::placeholders::_1));

        // 发布合并后的LaserScan话题
        pub_ = this->create_publisher<rm_interfaces::msg::NavigationMsg>("/nav/control", 10);
    }
    void Callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 在这里处理接收到的Twist消息
        // 例如：
        // 将线速度和角速度转换为你的控制指令
        // 创建一个LaserScan消息，填充数据
        sensor_msgs::msg::LaserScan scan;
        nav_msg.linear_velocity_x = msg->linear.x;
        nav_msg.linear_velocity_y = msg->linear.y;
        // ... 设置scan消息的各项参数 ...
        pub_->publish(nav_msg);
    }
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rm_interfaces::msg::NavigationMsg nav_msg;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cmd_vel2serial>());
    rclcpp::shutdown();
    return 0;
}
