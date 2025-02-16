#include <iostream>
#include <vector>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/odo_msg.hpp"




class OdoNode : public rclcpp::Node
{
public:
    OdoNode(const char *nodeName) : Node(nodeName)
    {
        // 接收nav/control的数据
        sub_ = this->create_subscription<rm_interfaces::msg::OdoMsg>(
            "/nav/odo", 10, std::bind(&OdoNode::callback, this, std::placeholders::_1));
        
        
    }

    ~OdoNode()
    {
    }

private:

    void callback(const rm_interfaces::msg::OdoMsg::SharedPtr msg)
    {

    }


    rclcpp::Subscription<rm_interfaces::msg::OdoMsg>::SharedPtr sub_;
    volatile float vx=0.0,vy=0.0,yaw=0.0;
    volatile float x=0.0,y=0.0;
    volatile long second = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto odo_node = std::make_shared<OdoNode>("odo_node");

    rclcpp::spin(odo_node);

    rclcpp::shutdown();
    return 0;
}
