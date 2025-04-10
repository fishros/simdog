#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/statistical_outlier_removal.h>  // 修复错误1
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

class NDTLocalization : public rclcpp::Node {
public:
    NDTLocalization() : Node("ndt_relocalization_node") {
        // 1. 加载全局地图 (PCD)
        global_map_ = std::make_shared<PointCloud>();
        if (pcl::io::loadPCDFile<PointT>("/home/lx/tmp/test_map/GlobalMap.pcd", *global_map_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file!");
            return;
        }

        // 2. 预处理全局地图：移除无效点
        PointCloud::Ptr filtered_map(new PointCloud);
        for (const auto& point : global_map_->points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                filtered_map->points.push_back(point);
            }
        }
        filtered_map->width = filtered_map->points.size();
        filtered_map->height = 1;
        filtered_map->is_dense = true;

        global_map_ = filtered_map;
        RCLCPP_INFO(this->get_logger(), "Loaded and filtered global map with %zu points", global_map_->size());
        
        ndt_.setResolution(1.0);  // 设置NDT网格分辨率（单位：米）
        ndt_.setInputTarget(global_map_);

        // 3. 订阅实时点云
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&NDTLocalization::cloudCallback, this, std::placeholders::_1));

        // 4. 发布位姿 (map -> base_link)
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ndt_pose", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ndt_odom", 10);

        // 5. TF 广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1. 转换 ROS2 PointCloud2 -> PCL PointCloud
        PointCloud::Ptr cloud(new PointCloud);
        pcl::fromROSMsg(*msg, *cloud);

        // 2. 严格预处理点云：移除无效点并进行统计滤波
        PointCloud::Ptr filtered_cloud(new PointCloud);
        
        // 移除 NaN 和无穷大点
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, indices);
        
        // 统计滤波，去除离群点
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(filtered_cloud);
        sor.setMeanK(50);  // 考虑50个最近邻点
        sor.setStddevMulThresh(1.0);  // 标准差阈值
        PointCloud::Ptr cleaned_cloud(new PointCloud);
        sor.filter(*cleaned_cloud);

        // 检查点云是否为空
        if (cleaned_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Filtered point cloud is empty!");
            return;
        }

        // 3. 执行 NDT 配准
        ndt_.setInputSource(cleaned_cloud);
        pcl::PointCloud<PointT> output;
        
        try {
            // 设置更宽松的 NDT 参数
            ndt_.setResolution(0.5);  // 网格分辨率
            ndt_.setStepSize(0.05);    // 优化步长
            ndt_.setTransformationEpsilon(0.01);  // 收敛阈值
            ndt_.setMaximumIterations(100);        // 最大迭代次数

            ndt_.align(output);
        } catch (const pcl::PCLException& e) {
            RCLCPP_ERROR(this->get_logger(), "NDT alignment failed: %s", e.what());
            return;
        }

        // 检查配准是否成功
        if (!ndt_.hasConverged()) {
            RCLCPP_WARN(this->get_logger(), "NDT did not converge");
            return;
        }

        // 4. 获取位姿 (4x4 变换矩阵)
        Eigen::Matrix4f pose = ndt_.getFinalTransformation();

        // 5. 转换为 ROS2 Pose
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.header.frame_id = "map";

        // 提取平移和旋转
        pose_msg.pose.position.x = pose(0, 3);
        pose_msg.pose.position.y = pose(1, 3);
        pose_msg.pose.position.z = pose(2, 3);

        Eigen::Matrix3f rot = pose.block<3, 3>(0, 0);
        Eigen::Quaternionf quat(rot);
        pose_msg.pose.orientation.x = quat.x();
        pose_msg.pose.orientation.y = quat.y();
        pose_msg.pose.orientation.z = quat.z();
        pose_msg.pose.orientation.w = quat.w();

        // 6. 发布位姿
        pose_pub_->publish(pose_msg);

        // 7. 发布 TF (map -> base_link)
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = pose(0, 3);
        transform.transform.translation.y = pose(1, 3);
        transform.transform.translation.z = pose(2, 3);
        transform.transform.rotation = pose_msg.pose.orientation;
        tf_broadcaster_->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", pose(0, 3), pose(1, 3), pose(2, 3));
        // 8. 发布 Odometry
        nav_msgs::msg::Odometry odom;
        odom.header = pose_msg.header;
        odom.child_frame_id = "base_link";
        odom.pose.pose = pose_msg.pose;
        odom_pub_->publish(odom);
        RCLCPP_INFO(this->get_logger(), 
            "NDT converged: %d, score: %f, points: %zu", 
            ndt_.hasConverged(), ndt_.getFitnessScore(), cleaned_cloud->size());
    }

    pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
    PointCloud::Ptr global_map_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NDTLocalization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}