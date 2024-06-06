#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>
#include <sstream>

class ClusteringNode : public rclcpp::Node
{
public:
  ClusteringNode() : Node("clustering_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensing/merged/points_downsampled", 10, std::bind(&ClusteringNode::topic_callback, this, std::placeholders::_1));
    cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/merged/clusters", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/sensing/merged/cluster_markers", 10);

    declare_parameter<double>("cluster_tolerance", 0.2);
    get_parameter("cluster_tolerance", cluster_tolerance_);

    declare_parameter<int>("min_cluster_size", 50);
    get_parameter("min_cluster_size", min_cluster_size_);

    declare_parameter<int>("max_cluster_size", 3000);
    get_parameter("max_cluster_size", max_cluster_size_);
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    try
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

      // Convert ROS message to PCL data type
      pcl::fromROSMsg(*msg, *cloud);

      // Check for NaNs and Infs in the point cloud
      for (const auto& point : cloud->points)
      {
        if (!pcl::isFinite(point))
        {
          RCLCPP_ERROR(this->get_logger(), "Point cloud contains NaNs or Infs");
          return;
        }
      }

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

        if (cloud_cluster->points.empty())
        {
          continue;
        }

        try
        {
          pcl::PCA<pcl::PointXYZ> pca;
          pca.setInputCloud(cloud_cluster);
          Eigen::Vector3f eigen_values = pca.getEigenValues();
          Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
          Eigen::Vector4f centroid;
          pcl::compute3DCentroid(*cloud_cluster, centroid);

          Eigen::Matrix4f projection_transform(Eigen::Matrix4f::Identity());
          projection_transform.block<3, 3>(0, 0) = eigen_vectors.transpose();
          projection_transform.block<3, 1>(0, 3) = -1.0f * (eigen_vectors.transpose() * centroid.head<3>());

          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pca_projected(new pcl::PointCloud<pcl::PointXYZ>());

          for (size_t i = 0; i < cloud_cluster->points.size(); ++i)
          {
            auto& point = cloud_cluster->points[i];
            Eigen::Vector4f point_homogeneous(point.x, point.y, point.z, 1.0f);

            Eigen::Vector4f point_transformed = projection_transform * point_homogeneous;
            pcl::PointXYZ transformed_point;
            transformed_point.x = point_transformed.x();
            transformed_point.y = point_transformed.y();
            transformed_point.z = point_transformed.z();
            cloud_pca_projected->points.push_back(transformed_point);
          }

          cloud_pca_projected->width = cloud_pca_projected->points.size();
          cloud_pca_projected->height = 1;
          cloud_pca_projected->is_dense = true;

          pcl::PointXYZ min_pt, max_pt;
          pcl::getMinMax3D(*cloud_pca_projected, min_pt, max_pt);

          Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());
          Eigen::Quaternionf qfinal(eigen_vectors);
          Eigen::Vector3f tfinal = eigen_vectors * mean_diag + centroid.head<3>();

          visualization_msgs::msg::Marker marker;
          marker.header.frame_id = msg->header.frame_id;
          marker.header.stamp = this->now();
          marker.ns = "cluster_markers";
          marker.id = cluster_id;
          marker.type = visualization_msgs::msg::Marker::CUBE;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.pose.position.x = tfinal.x();
          marker.pose.position.y = tfinal.y();
          marker.pose.position.z = tfinal.z();
          marker.pose.orientation.x = qfinal.x();
          marker.pose.orientation.y = qfinal.y();
          marker.pose.orientation.z = qfinal.z();
          marker.pose.orientation.w = qfinal.w();
          marker.scale.x = max_pt.x - min_pt.x;
          marker.scale.y = max_pt.y - min_pt.y;
          marker.scale.z = max_pt.z - min_pt.z;
          marker.color.a = 0.5;  // Transparency
          marker.color.r = 0.0;
          marker.color.g = 1.0;  // Green color for the bounding box
          marker.color.b = 0.0;

          marker_array.markers.push_back(marker);

          cluster_id++;
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(this->get_logger(), "Exception caught in PCA calculation for cluster %d: %s", cluster_id, e.what());
        }
        catch (...)
        {
          RCLCPP_ERROR(this->get_logger(), "Unknown exception caught in PCA calculation for cluster %d", cluster_id);
        }
      }

      marker_publisher_->publish(marker_array);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception caught in topic_callback: %s", e.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception caught in topic_callback");
    }
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
