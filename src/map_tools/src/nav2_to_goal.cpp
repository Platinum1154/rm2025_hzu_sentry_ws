#include <memory>
#include "nav2_msgs/action/navigate_to_pose.hpp" // 导入导航动作消息的头文件
#include "rclcpp/rclcpp.hpp"                     // 导入ROS 2的C++客户端库
#include "rclcpp_action/rclcpp_action.hpp"       // 导入ROS 2的C++ Action客户端库
#include <iostream>                              // std::cout, std::endl
#include <thread>                                // std::this_thread::sleep_for
#include <chrono>                                // std::chrono::seconds
#include "rm_interfaces/msg/decision.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
using NavigationAction = nav2_msgs::action::NavigateToPose; // 定义导航动作类型为NavigateToPose
volatile int naving_flag = 0;                               // 是否在导航
volatile int nav_statue = 0;


auto chassis_spin = std_msgs::msg::Float32();
auto gimbal_mode = std_msgs::msg::UInt8();
//int target_point_num = 0;

// 中心增益点
float target_x = 4.4f;                                                 // 目标点x坐标
float target_y = -2.4f;                                                 // 目标点y坐标
float rush[2] ={6.99,-0.2};
float target_point[8][2] = {{5.03,-1.63},   // 1
                            {5.04,-2.08},   // 2
                            {4.75,-2.86},   // 3
                            {4.15,-2.77},   // 6
                            {3.37,-2.86},   // 9
                            {3.54,-2.16},   // 8
                            {3.61,-1.49},   // 7
                            {4.23,-1.50},   // 4
                            };
// 补给点
float supply_x = -0.4f;                                               // 补给点x坐标（示例值）
float supply_y = 0.4f;                                                 // 补给点y坐标（示例值）
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_gimbal_mode;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_chassis_spin;

class NavToPoseClient : public rclcpp::Node
{
public:
    using NavigationActionClient = rclcpp_action::Client<NavigationAction>;               // 定义导航动作客户端类型
    using NavigationActionGoalHandle = rclcpp_action::ClientGoalHandle<NavigationAction>; // 定义导航动作目标句柄类型

    NavToPoseClient() : Node("nav_to_pose_client")
    {
        // 创建导航动作客户端
        action_client_ = rclcpp_action::create_client<NavigationAction>(
            this, "navigate_to_pose");

        // 创建决策消息的订阅者
        decision_subscriber_ = this->create_subscription<rm_interfaces::msg::Decision>(
            "nav/decision", // 替换为实际话题名称
            10,
            std::bind(&NavToPoseClient::decisionCallback, this, std::placeholders::_1));
            pub_chassis_spin = this->create_publisher<std_msgs::msg::Float32>("/nav/chassis_spin", 10);
            pub_gimbal_mode = this->create_publisher<std_msgs::msg::UInt8>("/nav/gimbal_mode", 10);
    }

    void decisionCallback(const rm_interfaces::msg::Decision::SharedPtr msg)
    {
        // 更新决策数据
        decision_data_ = *msg; // 将接收到的消息拷贝到成员变量中
    }

    void sendGoal(float x, float y)
    {
        naving_flag = 1; // 设置导航标志为正在导航
        // 等待导航动作服务器上线，等待时间为5秒
        while (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_INFO(get_logger(), "等待Action服务上线。");
        }
        // 设置导航目标点
        auto goal_msg = NavigationAction::Goal();
        goal_msg.pose.header.frame_id = "map"; // 设置目标点的坐标系为地图坐标系
        goal_msg.pose.pose.position.x = x;     // 使用传入的x坐标
        goal_msg.pose.pose.position.y = y;     // 使用传入的y坐标

        auto send_goal_options = rclcpp_action::Client<NavigationAction>::SendGoalOptions();
        // 设置请求目标结果回调函数
        send_goal_options.goal_response_callback =
            [this](NavigationActionGoalHandle::SharedPtr goal_handle)
        {
            if (goal_handle)
            {
                RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
                current_goal_handle_ = goal_handle; // 保存目标句柄
            }
        };

        // 设置执行结果回调函数
        send_goal_options.result_callback =
            [this](const NavigationActionGoalHandle::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "处理成功！");
                naving_flag = 0; // 设置导航标志为完成
            }
        };

        // 发送导航目标点
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void cancelGoal()
    {
        if (current_goal_handle_)
        {
            action_client_->async_cancel_goal(current_goal_handle_); // 取消目标点
            RCLCPP_INFO(this->get_logger(), "目标点已被取消");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "没有目标点可以取消");
        }
    }

    NavigationActionClient::SharedPtr action_client_;
    rclcpp::Subscription<rm_interfaces::msg::Decision>::SharedPtr decision_subscriber_; // 决策消息的订阅者
    rm_interfaces::msg::Decision decision_data_;                                        // 存储接收到的决策数据
    NavigationActionGoalHandle::SharedPtr current_goal_handle_;                         // 当前目标句柄
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavToPoseClient>();
    // 等待比赛开始的循环
    while (rclcpp::ok() && node->decision_data_.match_progress != 4)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 每隔1秒检查一次
        std::cout << "等待比赛开始..." << std::endl;
        rclcpp::spin_some(node); // 处理回调
    }

    // 等待导航完成
    // int timeout = 10; // 超时时间为20秒

    // while (naving_flag && timeout > 0)
    // {
    //     std::cout << "导航中，剩余时间：" << timeout << "秒" << std::endl;
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    //     rclcpp::spin_some(node); // 处理回调
    //     timeout--;
    // }

    // if (timeout <= 0)
    // {
    //     std::cout << "导航超时，未到达目标点！" << std::endl;

    //     // 取消目标点
    //     node->cancelGoal(); // 调用取消目标的方法
    // }
    // 第一阶段成功后进行第二阶段决策
    // 判断自身血量是否低于100
    // 循环判断血量状态
    while (rclcpp::ok())
    {
        // 每隔1秒检查一次血量
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 重新获取最新的决策数据
        // 这里假设你有一个方法可以更新决策数据
        // node->updateDecisionData();
        // RCLCPP_INFO(node->get_logger(), "血量：(%d)", node->decision_data_.self_sentry_hp);
        if (node->decision_data_.self_sentry_hp < 200)
        {
            nav_statue = 1;
            gimbal_mode.data = 0;
            pub_gimbal_mode->publish(gimbal_mode);
            RCLCPP_INFO(node->get_logger(), "血量低于200，返回补给点(%f, %f)", supply_x, supply_y);
            node->sendGoal(supply_x, supply_y); // 调用返回补给点的方法
        }
        else if (node->decision_data_.self_sentry_hp > 300 && node->decision_data_.match_progress == 4)
        {
            nav_statue = 2;
            gimbal_mode.data = 0;
            pub_gimbal_mode->publish(gimbal_mode);
            // 血量高于100，保持原地；
            //target_point_num++;
            // if(target_point_num>=8)target_point_num=0;
            // RCLCPP_INFO(node->get_logger(), "血量高于300,去增益点，(%f, %f)", target_point[target_point_num][0], target_point[target_point_num][1]);
            // node->sendGoal(target_point[target_point_num][0], target_point[target_point_num][1]); // 调用导航方法
            RCLCPP_INFO(node->get_logger(), "血量高于300,去增益点，(%f, %f)", target_x, target_y);
            node->sendGoal(target_x, target_y); // 调用导航方法
        }
        //小陀螺单独控制
        if(node->decision_data_.remain_time > 295 || node->decision_data_.match_progress != 4){
            chassis_spin.data = 0;
            pub_chassis_spin->publish(chassis_spin);
        }
        else{
            chassis_spin.data = 80;
            pub_chassis_spin->publish(chassis_spin);
        }
        rclcpp::spin_some(node);
        // 等待导航完成
        // timeout = 10; // 超时时间为10秒

        // while (naving_flag && timeout > 0)
        // {
        //     std::cout << "导航中，剩余时间：" << timeout << "秒" << std::endl;
        //     std::this_thread::sleep_for(std::chrono::seconds(1));
        //     rclcpp::spin_some(node); // 处理回调
        //     timeout--;
        // }

        // if (timeout <= 0)
        // {
        //     std::cout << "导航超时，未到达目标点！" << std::endl;

        //     // 取消目标点
        //     node->cancelGoal(); // 调用取消目标的方法

        //     // 处理回调
        //     rclcpp::spin_some(node);
        // }
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}