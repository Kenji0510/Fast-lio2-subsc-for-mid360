class FastLIOListener : public rclcpp::Node
{
public:
    FastLIOListener()
        : Node("fast_lio_listener"), repeat_count_cloud_registered_body_(0), repeat_count_laser_map_(0)
    {
        subscription_cloud_registered_body_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud_registered_body",
            5,
            std::bind(&FastLIOListener::callback_cloud_registered_body, this, std::placeholders::_1));

        subscription_laser_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "Laser_map",
            5,
            std::bind(&FastLIOListener::callback_laser_map, this, std::placeholders::_1));
    }

private:
    void callback_cloud_registered_body(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        process_pointcloud(msg, "cloud_registered_body", repeat_count_cloud_registered_body_);
    }

    void callback_laser_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        process_pointcloud(msg, "Laser_map", repeat_count_laser_map_);
    }

    void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &topic_name, size_t &repeat_count)
    {
        repeat_count++;

        if (!msg)
        {
            RCLCPP_WARN(this->get_logger(), "Received NULL pointcloud message on topic: %s", topic_name.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received pointcloud num: %d on topic: %s", msg->width * msg->height, topic_name.c_str());

        if (repeat_count % 30 == 0)
        {
            save_pointcloud(msg, topic_name, repeat_count);
            return;
        }
    }

    void voxelization(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &pcl_downsampled_cloud, const std::string &topic_name, size_t repeat_count)
    {
        double voxel_size = 0.05f;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);

        auto start_time = std::chrono::high_resolution_clock::now();
        voxel_grid.filter(*pcl_downsampled_cloud);
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
        std::cout << "\e[35m"
                  << "Voxelization times by PCL " << elapsed_time.count() << " ms on topic: " << topic_name << "\e[m" << std::endl;

        std::string file_name = "/home/kenji/pcd_data/fast-lio2/" + topic_name + "/voxelization_data_" + std::to_string(voxel_size) + "_" + std::to_string(repeat_count) + ".pcd";
        if (pcl::io::savePCDFileASCII(file_name, *pcl_downsampled_cloud) == -1)
        {
            RCLCPP_WARN(this->get_logger(), "Couldn't save voxelized pointcloud to %s", file_name.c_str());
            return;
        }
    }

    void save_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &topic_name, size_t repeat_count)
    {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        auto pcl_downsampled_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *cloud);

        voxelization(cloud, pcl_downsampled_cloud, topic_name, repeat_count);

        std::string file_name = "/home/kenji/pcd_data/fast-lio2/" + topic_name + "/" + topic_name + "_" + std::to_string(repeat_count) + ".pcd";
        if (pcl::io::savePCDFileASCII(file_name, *cloud) == -1)
        {
            RCLCPP_WARN(this->get_logger(), "Couldn't save pointcloud to %s", file_name.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Saved pointcloud to %s", file_name.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_cloud_registered_body_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_laser_map_;
    size_t repeat_count_cloud_registered_body_;
    size_t repeat_count_laser_map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastLIOListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
