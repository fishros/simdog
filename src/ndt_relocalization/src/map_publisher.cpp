#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

class MapPublisher : public rclcpp::Node {
public:
    MapPublisher() : Node("map_publisher") {
        // 1. 加载全局地图 (PCD)
        PointCloud::Ptr global_map(new PointCloud);
        if (pcl::io::loadPCDFile<PointT>("/home/lx/tmp/map_indoor1/GlobalMap.pcd", *global_map) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map->size());

        // 2. 转换并发布地图
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", 1);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),  // 0.1秒发布一次（确保RViz2能接收到）
            [this, global_map]() {
                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(*global_map, msg);
                msg.header.stamp = this->now();
                msg.header.frame_id = "map";  // 坐标系设为 map
                map_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Published global map");
            }
        );
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}