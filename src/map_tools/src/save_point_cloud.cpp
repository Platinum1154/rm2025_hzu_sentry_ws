#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class PointCloudSaver : public rclcpp::Node {
public:
    PointCloudSaver() : Node("pointcloud_saver") {
        // 订阅 PointCloud2 话题
        subscription1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud_1", 10,
            std::bind(&PointCloudSaver::pointCloudCallback_1, this, std::placeholders::_1));
        subscription2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud_2", 10,
            std::bind(&PointCloudSaver::pointCloudCallback_2, this, std::placeholders::_1));
        subscription3_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud_merged", 10,
            std::bind(&PointCloudSaver::pointCloudCallback_3, this, std::placeholders::_1));
    }

private:
    void pointCloudCallback_1(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 将 PointCloud2 转换为 PCL 的 PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud_1;
        pcl::fromROSMsg(*msg, cloud_1);

        // 保存到 PCD 文件
        pcl::io::savePCDFileASCII("point_cloud_1.pcd", cloud_1);
        RCLCPP_INFO(this->get_logger(), "Saved %zu data points to output.pcd.", cloud_1.points.size());
    }
    void pointCloudCallback_2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 将 PointCloud2 转换为 PCL 的 PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud_2;
        pcl::fromROSMsg(*msg, cloud_2);

        // 保存到 PCD 文件
        pcl::io::savePCDFileASCII("point_cloud_2.pcd", cloud_2);
        RCLCPP_INFO(this->get_logger(), "Saved %zu data points to output.pcd.", cloud_2.points.size());
    }
    void pointCloudCallback_3(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 将 PointCloud2 转换为 PCL 的 PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud_3;
        pcl::fromROSMsg(*msg, cloud_3);

        // 保存到 PCD 文件
        pcl::io::savePCDFileASCII("point_cloud_3.pcd", cloud_3);
        RCLCPP_INFO(this->get_logger(), "Saved %zu data points to output.pcd.", cloud_3.points.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription2_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription3_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSaver>());
    rclcpp::shutdown();
    return 0;
}
