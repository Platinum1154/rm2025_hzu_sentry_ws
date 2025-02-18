#include <memory>
#include "nav2_msgs/action/navigate_to_pose.hpp"  // 导入导航动作消息的头文件
#include "rclcpp/rclcpp.hpp"  // 导入ROS 2的C++客户端库
#include "rclcpp_action/rclcpp_action.hpp"  // 导入ROS 2的C++ Action客户端库
#include <iostream>       // std::cout, std::endl
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

using NavigationAction = nav2_msgs::action::NavigateToPose;  // 定义导航动作类型为NavigateToPose
volatile int naving_flag = 0;   // 是否在导航
volatile int rubbish = 0;

class NavToPoseClient : public rclcpp::Node {
public:
    using NavigationActionClient = rclcpp_action::Client<NavigationAction>;  // 定义导航动作客户端类型
    using NavigationActionGoalHandle =
        rclcpp_action::ClientGoalHandle<NavigationAction>;  // 定义导航动作目标句柄类型

    NavToPoseClient() : Node("nav_to_pose_client") {
        // 创建导航动作客户端
        action_client_ = rclcpp_action::create_client<NavigationAction>(
            this, "navigate_to_pose");
    }

    // 修改sendGoal函数，添加x和y坐标作为参数
    void sendGoal(float x, float y) {
        naving_flag=1;
        // 等待导航动作服务器上线，等待时间为5秒
        while (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_INFO(get_logger(), "等待Action服务上线。");
        }
        // 设置导航目标点
        auto goal_msg = NavigationAction::Goal();
        goal_msg.pose.header.frame_id = "map";  // 设置目标点的坐标系为地图坐标系
        goal_msg.pose.pose.position.x = x;  // 使用传入的x坐标
        goal_msg.pose.pose.position.y = y;  // 使用传入的y坐标

        auto send_goal_options =
            rclcpp_action::Client<NavigationAction>::SendGoalOptions();
        // 设置请求目标结果回调函数
        send_goal_options.goal_response_callback =
            [this](NavigationActionGoalHandle::SharedPtr goal_handle) {
            if (goal_handle) {
                RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
            }
        };
        // 设置移动过程反馈回调函数
        send_goal_options.feedback_callback =
            [this](
                NavigationActionGoalHandle::SharedPtr goal_handle,
                const std::shared_ptr<const NavigationAction::Feedback> feedback) {
            (void)goal_handle;  // 假装调用，避免 warning: unused
            //RCLCPP_INFO(this->get_logger(), "反馈剩余距离:%f",
            //            feedback->distance_remaining);
        };
        // 设置执行结果回调函数
        send_goal_options.result_callback =
            [this](const NavigationActionGoalHandle::WrappedResult& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "处理成功！");
                naving_flag=0;
            }
        };
        // 发送导航目标点
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    NavigationActionClient::SharedPtr action_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavToPoseClient>();

    // 提供x和y坐标
    float target_x1 = -1.9714107453768241f;//-7.279947020176306f
    float target_y1 = -2.1101602610421963f;//0.7360424262494546f
    std::cout << "send first point" << std::endl;
    // 调用sendGoal函数并传入目标坐标
    node->sendGoal(target_x1, target_y1);

    std::cout << "start sleep" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "stop sleep" << std::endl;

    int timeout = 20; // 超时时间为60秒
    while (naving_flag && timeout > 0) {
        std::cout << "i m here, i will sleep 1 seconds!" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        rclcpp::spin_some(node);
        timeout--;
    }
    if (timeout <= 0) {
        std::cout << "导航超时，未到达目标点！" << std::endl;
    }

    std::cout << "out of while" << std::endl;
    float target_x2 = -0.22028685810277354f;
    float target_y2 = 0.1153494676282539f;
    // 调用sendGoal函数并传入目标坐标
    node->sendGoal(target_x2, target_y2);
    std::cout << "send second point" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}