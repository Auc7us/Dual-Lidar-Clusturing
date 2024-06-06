#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>

class ClusteringNode : public rclcpp::Node
{
public:
  ClusteringNode() : Node("clustering_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensing/merged/points_downsampled", 10, std::bind(&ClusteringNode::topic_callback, this, std::placeholders::_1));
    cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/merged/clusters", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/sensing/merged/cluster_markers", 10);

    // Parameter for cluster tolerance (distance between points to consider them as part of the same cluster)
    declare_parameter<double>("cluster_tolerance", 0.2);
    get_parameter("cluster_tolerance", cluster_tolerance_);

    // Parameter for minimum cluster size (minimum number of points in a valid cluster)
    declare_parameter<int>("min_cluster_size", 50);
    get_parameter("min_cluster_size", min_cluster_size_);

    // Parameter for maximum cluster size (maximum number of points in a valid cluster)
    declare_parameter<int>("max_cluster_size", 3000);
    get_parameter("max_cluster_size", max_cluster_size_);
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Convert ROS message to PCL data type
    pcl::fromROSMsg(*msg, *cloud);

    // Perform Euclidean Cluster Extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);  // Cluster tolerance in meters
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    visualization_msgs::msg::MarkerArray marker_array;

    int cluster_id = 0;
    for (const auto& indices : cluster_indices)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());

      for (const auto& idx : indices.indices)
      {
        pcl::PointXYZRGB point;
        point.x = cloud->points[idx].x;
        point.y = cloud->points[idx].y;
        point.z = cloud->points[idx].z;
        point.r = 255;  // Assign red color to cluster points
        point.g = 0;
        point.b = 0;
        colored_cluster->points.push_back(point);
        colored_cloud->points.push_back(point);
        cloud_cluster->points.push_back(cloud->points[idx]);
      }

      // Check if cloud_cluster is empty before computing the bounding box
      if (cloud_cluster->points.empty())
      {
        RCLCPP_WARN(this->get_logger(), "Cluster %d is empty. Skipping bounding box calculation.", cluster_id);
        continue;
      }

      // Create a bounding box around the cluster
      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

      // Debugging: Print marker details
      RCLCPP_INFO(this->get_logger(), "Cluster ID: %d, Min: [%f, %f, %f], Max: [%f, %f, %f]", cluster_id, min_pt.x(), min_pt.y(), min_pt.z(), max_pt.x(), max_pt.y(), max_pt.z());

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = msg->header.frame_id;
      marker.header.stamp = this->now();
      marker.ns = "cluster_markers";
      marker.id = cluster_id;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2;
      marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2;
      marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2;
      marker.scale.x = max_pt.x() - min_pt.x();
      marker.scale.y = max_pt.y() - min_pt.y();
      marker.scale.z = max_pt.z() - min_pt.z();
      marker.color.a = 0.5;  // Transparency
      marker.color.r = 0.0;
      marker.color.g = 1.0;  // Green color for the bounding box
      marker.color.b = 0.0;

      marker_array.markers.push_back(marker);

      cluster_id++;
    }

    // Convert clustered PCL data back to ROS message
    sensor_msgs::msg::PointCloud2 cluster_output;
    pcl::toROSMsg(*colored_cloud, cluster_output);
    cluster_output.header = msg->header;
    cluster_publisher_->publish(cluster_output);
    marker_publisher_->publish(marker_array);
  }

  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClusteringNode>());
  rclcpp::shutdown();
  return 0;
}















// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/common/common.h>

// class ClusteringNode : public rclcpp::Node
// {
// public:
//   ClusteringNode() : Node("clustering_node")
//   {
//     subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//         "/sensing/x90_l/points_downsampled", 10, std::bind(&ClusteringNode::topic_callback, this, std::placeholders::_1));
//     cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/x90_l/clusters", 10);
//     marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/sensing/x90_l/cluster_markers", 10);

//     // Parameter for cluster tolerance (distance between points to consider them as part of the same cluster)
//     declare_parameter<double>("cluster_tolerance", 0.2);
//     get_parameter("cluster_tolerance", cluster_tolerance_);

//     // Parameter for minimum cluster size (minimum number of points in a valid cluster)
//     declare_parameter<int>("min_cluster_size", 50);
//     get_parameter("min_cluster_size", min_cluster_size_);

//     // Parameter for maximum cluster size (maximum number of points in a valid cluster)
//     declare_parameter<int>("max_cluster_size", 3000);
//     get_parameter("max_cluster_size", max_cluster_size_);
//   }

// private:
//   void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
//   {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

//     // Convert ROS message to PCL data type
//     pcl::fromROSMsg(*msg, *cloud);

//     // Perform Euclidean Cluster Extraction
//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//     tree->setInputCloud(cloud);

//     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//     ec.setClusterTolerance(cluster_tolerance_);  // Cluster tolerance in meters
//     ec.setMinClusterSize(min_cluster_size_);
//     ec.setMaxClusterSize(max_cluster_size_);
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(cloud);
//     ec.extract(cluster_indices);

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
//     visualization_msgs::msg::MarkerArray marker_array;

//     int cluster_id = 0;
//     for (const auto& indices : cluster_indices)
//     {
//       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>());
//       pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());

//       for (const auto& idx : indices.indices)
//       {
//         pcl::PointXYZRGB point;
//         point.x = cloud->points[idx].x;
//         point.y = cloud->points[idx].y;
//         point.z = cloud->points[idx].z;
//         point.r = 255;  // Assign red color to cluster points
//         point.g = 0;
//         point.b = 0;
//         colored_cluster->points.push_back(point);
//         colored_cloud->points.push_back(point);
//         cloud_cluster->points.push_back(cloud->points[idx]);
//       }

//       // Check if cloud_cluster is empty before computing the bounding box
//       if (cloud_cluster->points.empty())
//       {
//         RCLCPP_WARN(this->get_logger(), "Cluster %d is empty. Skipping bounding box calculation.", cluster_id);
//         continue;
//       }

//       // Create a bounding box around the cluster
//       Eigen::Vector4f min_pt, max_pt;
//       pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

//       // Debugging: Print marker details
//       RCLCPP_INFO(this->get_logger(), "Cluster ID: %d, Min: [%f, %f, %f], Max: [%f, %f, %f]", cluster_id, min_pt.x(), min_pt.y(), min_pt.z(), max_pt.x(), max_pt.y(), max_pt.z());

//       visualization_msgs::msg::Marker marker;
//       marker.header.frame_id = msg->header.frame_id;
//       marker.header.stamp = this->now();
//       marker.ns = "cluster_markers";
//       marker.id = cluster_id;
//       marker.type = visualization_msgs::msg::Marker::CUBE;
//       marker.action = visualization_msgs::msg::Marker::ADD;
//       marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2;
//       marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2;
//       marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2;
//       marker.scale.x = max_pt.x() - min_pt.x();
//       marker.scale.y = max_pt.y() - min_pt.y();
//       marker.scale.z = max_pt.z() - min_pt.z();
//       marker.color.a = 0.5;  // Transparency
//       marker.color.r = 0.0;
//       marker.color.g = 1.0;  // Green color for the bounding box
//       marker.color.b = 0.0;

//       marker_array.markers.push_back(marker);

//       cluster_id++;
//     }

//     // Convert clustered PCL data back to ROS message
//     sensor_msgs::msg::PointCloud2 cluster_output;
//     pcl::toROSMsg(*colored_cloud, cluster_output);
//     cluster_output.header = msg->header;
//     cluster_publisher_->publish(cluster_output);
//     marker_publisher_->publish(marker_array);
//   }

//   double cluster_tolerance_;
//   int min_cluster_size_;
//   int max_cluster_size_;

//   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
//   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_publisher_;
//   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<ClusteringNode>());
//   rclcpp::shutdown();
//   return 0;
// }














