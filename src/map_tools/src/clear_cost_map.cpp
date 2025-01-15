#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"  // 引用正确的服务头文件

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  // 初始化 ROS2 节点
  rclcpp::init(argc, argv);

  // 创建一个节点
  auto node = rclcpp::Node::make_shared("clear_costmap_client");

  // 创建服务客户端，注意使用正确的服务类型
  auto client = node->create_client<nav2_msgs::srv::ClearEntireCostmap>("local_costmap/clear_entirely_local_costmap");

  // 等待服务可用
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  // 构建请求，注意这是 ClearEntireCostmap 类型的请求
  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

  // 异步发送请求并等待结果
  auto result = client->async_send_request(request);

  // 等待服务返回结果
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Successfully cleared the local costmap.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service clear_entirely_local_costmap");
  }

  // 关闭 ROS2
  rclcpp::shutdown();
  return 0;
}
