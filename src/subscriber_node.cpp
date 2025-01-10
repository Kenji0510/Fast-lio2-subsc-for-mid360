// #include <ros/ros.h>
// #include <rclcpp/rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <pcl_conversions/pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <open3d/Open3D.h>
#include <pcl-1.12/pcl/io/pcd_io.h>
#include <pcl-1.12/pcl/point_cloud.h>
#include <pcl-1.12/pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <iostream>
#include <string>
#include <chrono>

class FastLIOListener : public rclcpp::Node
{
public:
    FastLIOListener() : Node("fast_lio_listener"), repeat_count_(0)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name_,
            5,
            std::bind(&FastLIOListener::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        repeat_count_++;

        if (!msg)
        {
            RCLCPP_WARN(this->get_logger(), "Received NULL pointcloud message!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received pointcloud num: %d", msg->width * msg->height);

        if (repeat_count_ % 30 == 0)
        {
            save_pointcloud(msg);
            return;
        }
    }

    void voxelization(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &pcl_downsampled_cloud)
    {

        double voxel_size = 0.05f;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);

        auto start_time = std::chrono::high_resolution_clock::now();
        voxel_grid.filter(*pcl_downsampled_cloud);
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
        std::cout << "\e[35m" << "Voxelization times by PCL " << elapsed_time.count() << " ms" << "\e[m" << std::endl;

        std::string file_name =
            "/home/kenji/pcd_data/fast-lio2/cloud_registered_body/voxelization_data/voxelization_data_" +
            std::to_string(voxel_size) +
            "_" + std::to_string(repeat_count_) + ".pcd";
        if (pcl::io::savePCDFileASCII(file_name + "", *pcl_downsampled_cloud) == -1)
        {
            RCLCPP_WARN(this->get_logger(), "Couldn't save pointcloud to %s", file_name.c_str());
            return;
        }
    }

    void save_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        auto pcl_downsampled_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *cloud);

        voxelization(cloud, pcl_downsampled_cloud);

        std::string file_name = "/home/kenji/pcd_data/fast-lio2/cloud_registered_body/" + topic_name_ + "_" + std::to_string(repeat_count_) + ".pcd";
        if (pcl::io::savePCDFileASCII(file_name, *cloud) == -1)
        {
            RCLCPP_WARN(this->get_logger(), "Couldn't save pointcloud to %s", file_name.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Saved pointcloud to %s", file_name.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    size_t repeat_count_;
    // std::string topic_name_ = "Laser_map";
    std::string topic_name_ = "cloud_registered_body";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastLIOListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}