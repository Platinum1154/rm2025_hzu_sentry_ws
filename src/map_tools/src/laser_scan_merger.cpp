#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <pcl/filters/radius_outlier_removal.h>


class LaserScanMerger : public rclcpp::Node {
public:
    LaserScanMerger() : Node("laser_scan_merger") {
        // 订阅两个 LaserScan 话题
        scan1_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan1", 10, std::bind(&LaserScanMerger::scan1Callback, this, std::placeholders::_1));
        scan2_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan2", 10, std::bind(&LaserScanMerger::scan2Callback, this, std::placeholders::_1));

        // 发布合并后的 LaserScan 话题
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/merged_scan", 10);
        point_cloud_1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_1", 10);
        point_cloud_2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_2", 10);
        point_cloud_merged_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_merged", 10);
    }

private:
    void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double x_offset, double y_offset, double z_offset) {
        for (auto& point : cloud->points) {
            point.x += x_offset;
            point.y += y_offset;
            point.z += z_offset;
        }
    }
    // 回调函数，处理第一个 LaserScan 话题
    void scan1Callback(const sensor_msgs::msg::LaserScan::SharedPtr scan1) {
        projector_.projectLaser(*scan1, cloud1_);  // 将 LaserScan 转换为 PointCloud2
        pcl::fromROSMsg(cloud1_, *pcl_cloud1_);    // 将 PointCloud2 转换为 PCL 点云
        point_cloud_1_pub_->publish(cloud1_);
        transformPointCloud(pcl_cloud1_, -0.22026, 0.2238, 0.0); // 对点云进行平移变换，x轴向左平移23cm
        mergeAndPublish();                         // 尝试合并点云并发布
    }

    // 回调函数，处理第二个 LaserScan 话题
void scan2Callback(const sensor_msgs::msg::LaserScan::SharedPtr scan2) {
    // 将 LaserScan 转换为 PointCloud2
    projector_.projectLaser(*scan2, cloud2_);

    // 将 PointCloud2 转换为 PCL 点云
    pcl::fromROSMsg(cloud2_, *pcl_cloud2_);

    // 创建旋转变换矩阵
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())); // 绕 Z 轴旋转 180 度

    // 应用旋转变换
    pcl::transformPointCloud(*pcl_cloud2_, *pcl_cloud2_, transform);

    // 转换回 PointCloud2
    pcl::toROSMsg(*pcl_cloud2_, cloud2_);

    // 发布点云
    point_cloud_2_pub_->publish(cloud2_);

    // 尝试合并点云并发布
    mergeAndPublish();
}
    // 合并两个点云并发布为新的 LaserScan
void mergeAndPublish() {
        if (pcl_cloud1_->empty() || pcl_cloud2_->empty()) {
            //RCLCPP_WARN(this->get_logger(), "One or both point clouds are empty, skipping merge.");
            return;  // 等待两个点云都接收到后再合并
        }
        //RCLCPP_INFO(this->get_logger(), "Point Cloud1 size: %zu", pcl_cloud1_->size());
        //RCLCPP_INFO(this->get_logger(), "Point Cloud2 size: %zu", pcl_cloud2_->size());

        // 合并两个点云
        pcl::PointCloud<pcl::PointXYZ> merged_pcl_cloud;
        merged_pcl_cloud += *pcl_cloud1_;
        merged_pcl_cloud += *pcl_cloud2_;

        // 打印合并后点云的大小
        // RCLCPP_INFO(this->get_logger(), "Before filter Point Cloud size: %zu", merged_pcl_cloud.size());
        
        //滤波器
        Eigen::Vector4f min_point(-0.35, -0.35, -1000.0, 1.0); // 正方形的最小点 (x_min, y_min, z_min)
        Eigen::Vector4f max_point(0.35, 0.35, 1000.0, 1.0);    // 正方形的最大点 (x_max, y_max, z_max)

        // 创建CropBox滤波器
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(merged_pcl_cloud.makeShared());
        crop_box_filter.setMin(min_point);
        crop_box_filter.setMax(max_point);
        crop_box_filter.setNegative(true); // 设置为true以保留不在正方形区域内的点

        // 过滤点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        crop_box_filter.filter(merged_pcl_cloud);


        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
        radius_filter.setInputCloud(merged_pcl_cloud.makeShared());  // 设置输入点云

        // 设置搜索半径和最小邻居数
        radius_filter.setRadiusSearch(0.1);  // 半径内搜索点的距离 (单位：米)
        radius_filter.setMinNeighborsInRadius(2);  // 在该半径内的最小点数，如果少于该数目则被认为是离群点并被移除

        // 执行滤波操作并将结果保存到 cloud_filtered 中
        radius_filter.filter(*cloud_filtered); 

        // 输出过滤后的点云大小
        // RCLCPP_INFO(this->get_logger(), "Filtered Point Cloud size: %zu", cloud_filtered->size());


        // 将合并后的点云转换回 PointCloud2
        sensor_msgs::msg::PointCloud2 merged_cloud_msg;
        pcl::toROSMsg(*cloud_filtered, merged_cloud_msg);
        merged_cloud_msg.header = cloud1_.header;  // 保持与第一个点云的时间戳和坐标系一致

        point_cloud_merged_pub_->publish(merged_cloud_msg);
        // 将合并后的 PointCloud2 转换为 LaserScan
        sensor_msgs::msg::LaserScan merged_scan;
        pointcloudToLaserScan(merged_cloud_msg, merged_scan);
        merged_scan.header.frame_id = "merged_laser";
        
        // 滤除0.0
        for(auto &range : merged_scan.ranges){
            if(range == 0.0){
                range = 3.0;
            }
        }


        // 发布合并后的 LaserScan
        scan_pub_->publish(merged_scan);
        // RCLCPP_INFO(this->get_logger(), "publish merged_scan");
    }
    // 从 PointCloud2 转换为 LaserScan
    void pointcloudToLaserScan(const sensor_msgs::msg::PointCloud2& cloud, sensor_msgs::msg::LaserScan& laser_scan) {
        // 示例：将 PointCloud2 转换为 LaserScan
        laser_scan.header = cloud.header;
        laser_scan.angle_min = -M_PI;
        laser_scan.angle_max = M_PI;
        laser_scan.angle_increment = 2 * M_PI / 360; // 假设360度扫描
        laser_scan.range_min = 0.0;
        laser_scan.range_max = 360.0;

        laser_scan.ranges.resize(360, std::numeric_limits<float>::infinity());

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);

    for (const auto& point : pcl_cloud.points) {
        // 计算点的角度
        float angle = atan2(point.y, point.x);
        // 将角度转换为范围数组的索引
        int index = static_cast<int>((angle - laser_scan.angle_min) / laser_scan.angle_increment);
        if (index >= 0 && index < 360) {
            // 计算点的距离
            float range = sqrt(point.x * point.x + point.y * point.y);
            // 仅在新距离小于当前值时更新距离（保留最近的障碍物）
            if (range < laser_scan.ranges[index]) {
                laser_scan.ranges[index] = range;
            }
        }
    }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan2_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_1_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_2_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_merged_pub_;

    laser_geometry::LaserProjection projector_;
    sensor_msgs::msg::PointCloud2 cloud1_, cloud2_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud1_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud2_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanMerger>());
    rclcpp::shutdown();
    return 0;
}
