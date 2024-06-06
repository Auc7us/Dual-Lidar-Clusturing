#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <iostream>

class LidarWarpingNode : public rclcpp::Node
{
public:
  LidarWarpingNode() : Node("lidar_warping_node")
  {
    subscription_l_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensing/x90_l/points", 10, std::bind(&LidarWarpingNode::topic_callback_l, this, std::placeholders::_1));
    subscription_r_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensing/x90_r/points", 10, std::bind(&LidarWarpingNode::topic_callback_r, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/merged/points", 10);

    // Define corresponding points (these should be carefully measured)
    L1 << 1.43006741,  3.75761,     0.07623895;
    L2 << 2.1734847,   3.42211517,  0.0758504;
    L3 << 2.13017787,  3.32969644, -0.52834832;
    L4 << 1.3878836,   3.6651615,  -0.52538595;

    R1 << -2.07404628, 3.36568131, 0.02462647;
    R2 << -1.34926085, 3.71491671, 0.03323364;
    R3 << -1.31849233, 3.66750068, -0.57201976;
    R4 << -2.04623712, 3.31706462, -0.57846992;

    compute_transformation_matrix();
  }

private:
  void topic_callback_l(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::fromROSMsg(*msg, *cloud_l_);
    cloud_l_received_ = true;
    merge_and_publish();
  }

  void topic_callback_r(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::fromROSMsg(*msg, *cloud_r_);
    cloud_r_received_ = true;
    merge_and_publish();
  }

  void merge_and_publish()
  {
    if (cloud_l_received_ && cloud_r_received_)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_l_warped(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::transformPointCloud(*cloud_l_, *cloud_l_warped, transform_matrix_.cast<float>());

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointXYZ>());
      *cloud_merged = *cloud_l_warped + *cloud_r_;

      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(*cloud_merged, output);
      output.header.frame_id = "map"; // Use the appropriate frame
      publisher_->publish(output);

      // Reset the flags
      cloud_l_received_ = false;
      cloud_r_received_ = false;
    }
  }

  void compute_transformation_matrix()
  {
    Eigen::Matrix<double, 4, 3> A, B;
    A << L1.transpose(), L2.transpose(), L3.transpose(), L4.transpose();
    B << R1.transpose(), R2.transpose(), R3.transpose(), R4.transpose();

    // Center the points
    Eigen::Vector3d centroid_A = A.colwise().mean();
    Eigen::Vector3d centroid_B = B.colwise().mean();
    Eigen::Matrix<double, 4, 3> AA = A.rowwise() - centroid_A.transpose();
    Eigen::Matrix<double, 4, 3> BB = B.rowwise() - centroid_B.transpose();

    // Compute H matrix
    Eigen::Matrix3d H = AA.transpose() * BB;

    // Singular Value Decomposition
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * U.transpose();

    // Ensure a right-handed coordinate system
    if (R.determinant() < 0)
    {
      V.col(2) *= -1;
      R = V * U.transpose();
    }

    Eigen::Vector3d t = centroid_B - R * centroid_A;

    // Form the transformation matrix
    transform_matrix_.setIdentity();
    transform_matrix_.block<3, 3>(0, 0) = R;
    transform_matrix_.block<3, 1>(0, 3) = t;

    // Debug: Print the final transformation matrix
    std::cout << "Transformation matrix: \n" << transform_matrix_ << std::endl;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_l_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_r_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_l_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  bool cloud_l_received_ = false;
  bool cloud_r_received_ = false;

  Eigen::Matrix4d transform_matrix_;
  Eigen::Vector3d L1, L2, L3, L4;
  Eigen::Vector3d R1, R2, R3, R4;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarWarpingNode>());
  rclcpp::shutdown();
  return 0;
}






















// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/common/transforms.h>
// #include <pcl/io/io.h>
// #include <Eigen/Dense>
// #include <iostream>

// class LidarWarpingNode : public rclcpp::Node
// {
// public:
//   LidarWarpingNode() : Node("lidar_warping_node")
//   {
//     subscription_l_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//         "/sensing/x90_l/points_downsampled", 10, std::bind(&LidarWarpingNode::topic_callback_l, this, std::placeholders::_1));
//     subscription_r_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//         "/sensing/x90_r/points_downsampled", 10, std::bind(&LidarWarpingNode::topic_callback_r, this, std::placeholders::_1));
//     publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/merged/points", 10);

//     R1 << -2.07404628, 3.36568131, 0.02462647;
//     R2 << -1.34926085, 3.71491671, 0.03323364;
//     R3 << -1.31849233, 3.66750068, -0.57201976;
//     R4 << -2.04623712, 3.31706462, -0.57846992;

//     L2 << 1.43006741,  3.75761,     0.07623895;
//     L1 << 2.1734847,   3.42211517,  0.0758504;
//     L4 << 2.13017787,  3.32969644, -0.52834832;
//     L3 << 1.3878836,   3.6651615,  -0.52538595;

//     compute_transformation_matrix();
//   }

// private:
//   void topic_callback_l(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//   {
//     pcl::fromROSMsg(*msg, *cloud_l_);
//     cloud_l_received_ = true;
//     merge_and_publish();
//   }

//   void topic_callback_r(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//   {
//     pcl::fromROSMsg(*msg, *cloud_r_);
//     cloud_r_received_ = true;
//     merge_and_publish();
//   }

//   void merge_and_publish()
//   {
//     if (cloud_l_received_ && cloud_r_received_)
//     {
//       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_l_warped(new pcl::PointCloud<pcl::PointXYZ>());
//       pcl::transformPointCloud(*cloud_l_, *cloud_l_warped, transform_matrix_.cast<float>());

//       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointXYZ>());
//       *cloud_merged = *cloud_l_warped + *cloud_r_;

//       sensor_msgs::msg::PointCloud2 output;
//       pcl::toROSMsg(*cloud_merged, output);
//       output.header.frame_id = "map"; // Use the appropriate frame
//       publisher_->publish(output);

//       // Reset the flags
//       cloud_l_received_ = false;
//       cloud_r_received_ = false;
//     }
//   }

//   void compute_transformation_matrix()
//   {
//     Eigen::MatrixXd A(4, 3), B(4, 3);
//     A << L1.transpose(), L2.transpose(), L3.transpose(), L4.transpose();
//     B << R1.transpose(), R2.transpose(), R3.transpose(), R4.transpose();
    
//     // Center the points
//     Eigen::Vector3d centroid_A = A.colwise().mean();
//     Eigen::Vector3d centroid_B = B.colwise().mean();
//     Eigen::MatrixXd AA = A.rowwise() - centroid_A.transpose();
//     Eigen::MatrixXd BB = B.rowwise() - centroid_B.transpose();
    
//     // Debug: Print centroids and centered points
//     std::cout << "Centroid A: " << centroid_A.transpose() << std::endl;
//     std::cout << "Centroid B: " << centroid_B.transpose() << std::endl;
//     std::cout << "AA: \n" << AA << std::endl;
//     std::cout << "BB: \n" << BB << std::endl;
    
//     // Compute H matrix
//     Eigen::Matrix3d H = BB.transpose() * AA;
    
//     // Debug: Print H matrix
//     std::cout << "H matrix: \n" << H << std::endl;
    
//     // Singular Value Decomposition
//     Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
//     Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
    
//     // Debug: Print SVD results
//     std::cout << "U matrix: \n" << svd.matrixU() << std::endl;
//     std::cout << "V matrix: \n" << svd.matrixV() << std::endl;
//     std::cout << "Rotation matrix: \n" << R << std::endl;
    
//     // Ensure a right-handed coordinate system
//     if (R.determinant() < 0)
//     {
//       R.col(2) *= -1;
//     }
    
//     Eigen::Vector3d t = centroid_B - R * centroid_A;
    
//     // Form the transformation matrix
//     transform_matrix_.setIdentity();
//     transform_matrix_.block<3, 3>(0, 0) = R;
//     transform_matrix_.block<3, 1>(0, 3) = t;

//     // Debug: Print the final transformation matrix
//     std::cout << "Transformation matrix: \n" << transform_matrix_ << std::endl;
//   }

//   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_l_;
//   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_r_;
//   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_l_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

//   bool cloud_l_received_ = false;
//   bool cloud_r_received_ = false;

//   Eigen::Matrix4d transform_matrix_;
//   Eigen::Vector3d L1, L2, L3, L4;
//   Eigen::Vector3d R1, R2, R3, R4;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<LidarWarpingNode>());
//   rclcpp::shutdown();
//   return 0;
// }








