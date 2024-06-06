#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class DownsamplingNode : public rclcpp::Node
{
public:
  DownsamplingNode() : Node("downsampling_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensing/merged/points", 10, std::bind(&DownsamplingNode::topic_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/merged/points_downsampled", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    process_point_cloud(msg, publisher_);
  }

  void process_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                           const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) const
  {
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    // Convert ROS message to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);

    // Perform voxel grid downsampling
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.07f, 0.07f, 0.07f);  // Example voxel size
    sor.filter(*cloud_filtered);

    // Convert filtered PCL data back to ROS message
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_filtered, output);

    publisher->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DownsamplingNode>());
  rclcpp::shutdown();
  return 0;
}
