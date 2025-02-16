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
        time = this->get_clock()->now().seconds();
        if(time-last_time<0.1 && msg->vx<10 && msg->vy<10 && msg->vx>-10 && msg->vy>-10 ){
            radians =msg->yaw * 3.141592653 / 180;
            vx = msg->vx*std::cos(radians)-msg->vy*std::sin(radians);
            vy = msg->vx*std::sin(radians)+msg->vy*std::cos(radians);
            x += vx * (time-last_time);
            y += vy * (time-last_time);
        }
        RCLCPP_INFO(this->get_logger(),"x:%f, y:%f\n",x,y);
        last_time = time;
    }


    rclcpp::Subscription<rm_interfaces::msg::OdoMsg>::SharedPtr sub_;
    volatile float vx=0.0,vy=0.0,yaw=0.0;
    volatile float x=0.0,y=0.0;
    volatile float radians=0.0;
    volatile double time,last_time;
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
