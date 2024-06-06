# ############# Open3d VOXEL

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, PointField
# from sensor_msgs_py import point_cloud2
# import numpy as np
# import open3d as o3d

# class LidarDownsamplingNode(Node):
#     def __init__(self):
#         super().__init__('lidar_downsampling_node')
#         self.subscription = self.create_subscription(
#             PointCloud2,
#             '/sensing/x90_l/points',
#             self.listener_callback,
#             10)
#         self.publisher_ = self.create_publisher(PointCloud2, '/sensing/x90_l/points_downsampled', 10)

#     def listener_callback(self, msg):
#         points = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))

#         # Extract the x, y, z components from the structured array
#         x = np.array([point[0] for point in points], dtype=np.float32)
#         y = np.array([point[1] for point in points], dtype=np.float32)
#         z = np.array([point[2] for point in points], dtype=np.float32)

#         # Combine x, y, z into a single numpy array
#         points_array = np.vstack((x, y, z)).T

#         # Create an Open3D point cloud
#         pcd = o3d.geometry.PointCloud()
#         pcd.points = o3d.utility.Vector3dVector(points_array)

#         # Downsample the point cloud using a voxel grid filter
#         downsampled_pcd = pcd.voxel_down_sample(voxel_size=0.07)  # Example voxel size

#         # Convert back to numpy array
#         downsampled_points = np.asarray(downsampled_pcd.points)

#         # Create a new PointCloud2 message for the downsampled points
#         header = msg.header
#         fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#         ]

#         downsampled_point_cloud_msg = point_cloud2.create_cloud(header, fields, downsampled_points)

#         # Publish the downsampled point cloud
#         self.publisher_.publish(downsampled_point_cloud_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarDownsamplingNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




































# ######                      NUMPY VOXEL
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, PointField
# from sensor_msgs_py import point_cloud2
# import numpy as np
# import time

# class LidarDownsamplingNode(Node):
#     def __init__(self):
#         super().__init__('lidar_downsampling_node')
#         self.subscription = self.create_subscription(
#             PointCloud2,
#             '/sensing/x90_l/points',
#             self.listener_callback,
#             10)
#         self.publisher_ = self.create_publisher(PointCloud2, '/sensing/x90_l/points_downsampled', 10)

#     def listener_callback(self, msg):
#         start_time = time.time()

#         points = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))

#         # Debug: Print the first few points to check their format
#         if points:
#             print("First few points:", points[:5])
#         else:
#             print("No points found in the message.")

#         # Extract x, y, z into separate arrays
#         x = np.array([p[0] for p in points], dtype=np.float32)
#         y = np.array([p[1] for p in points], dtype=np.float32)
#         z = np.array([p[2] for p in points], dtype=np.float32)

#         # Stack them into a single 2D array
#         points_array = np.vstack((x, y, z)).T

#         # Debug: Print the shape of the numpy array
#         print("Shape of points_array:", points_array.shape)

#         # Define voxel grid size
#         voxel_size = 0.05  # Example voxel size in meters

#         # Downsample the points using a voxel grid filter
#         downsampled_points = self.voxel_grid_filter(points_array, voxel_size)

#         # Create a new PointCloud2 message for the downsampled points
#         header = msg.header
#         fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#         ]

#         downsampled_point_cloud_msg = point_cloud2.create_cloud(header, fields, downsampled_points)

#         # Publish the downsampled point cloud
#         self.publisher_.publish(downsampled_point_cloud_msg)

#         end_time = time.time()
#         processing_time = end_time - start_time
#         print(f"Processing time: {processing_time:.4f} seconds")

#     def voxel_grid_filter(self, points, voxel_size):
#         # Compute the voxel grid indices
#         voxel_indices = np.floor(points / voxel_size).astype(np.int32)

#         # Create a unique voxel index for each point
#         voxel_keys = np.dot(voxel_indices, [1, 1e6, 1e12])

#         # Find the unique voxels and their first occurrence indices
#         unique_voxels, unique_indices = np.unique(voxel_keys, return_inverse=True)

#         # Create an array to hold the sums and counts for each voxel
#         voxel_sums = np.zeros((len(unique_voxels), 3), dtype=np.float32)
#         voxel_counts = np.zeros(len(unique_voxels), dtype=np.int32)

#         # Sum the points in each voxel
#         np.add.at(voxel_sums, unique_indices, points)
#         np.add.at(voxel_counts, unique_indices, 1)

#         # Calculate the mean for each voxel
#         non_empty_voxels = voxel_counts > 0
#         voxel_means = np.divide(voxel_sums[non_empty_voxels], voxel_counts[non_empty_voxels][:, None])

#         return voxel_means

# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarDownsamplingNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




























######                      ANGULAR



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, PointField
# from sensor_msgs_py import point_cloud2
# import numpy as np

# class LidarDownsamplingNode(Node):
#     def __init__(self):
#         super().__init__('lidar_downsampling_node')
#         self.subscription = self.create_subscription(
#             PointCloud2,
#             '/sensing/x90_l/points',
#             self.listener_callback,
#             10)
#         self.publisher_ = self.create_publisher(PointCloud2, '/sensing/x90_l/points_downsampled', 10)

#     def listener_callback(self, msg):
#         points = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))

#         # Debug: Print the first few points to check their format
#         if points:
#             print("First few points:", points[:5])
#         else:
#             print("No points found in the message.")

#         # Extract x, y, z into separate arrays
#         x = np.array([p[0] for p in points], dtype=np.float32)
#         y = np.array([p[1] for p in points], dtype=np.float32)
#         z = np.array([p[2] for p in points], dtype=np.float32)

#         # Stack them into a single 2D array
#         points_array = np.vstack((x, y, z)).T

#         # Debug: Print the shape of the numpy array
#         print("Shape of points_array:", points_array.shape)

#         # Define the angular resolution for the grid
#         azimuth_resolution = np.deg2rad(0.7)  # 1 degree resolution
#         elevation_resolution = np.deg2rad(0.7)  # 1 degree resolution

#         # Downsample the points
#         downsampled_points = self.downsample_lidar(points_array, (azimuth_resolution, elevation_resolution))

#         # Create a new PointCloud2 message for the downsampled points
#         header = msg.header
#         fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#         ]

#         downsampled_point_cloud_msg = point_cloud2.create_cloud(header, fields, downsampled_points)

#         # Publish the downsampled point cloud
#         self.publisher_.publish(downsampled_point_cloud_msg)

#     def downsample_lidar(self, points, resolution):
#         grid = {}

#         for point in points:
#             x, y, z = point[:3]
#             azimuth = np.arctan2(y, x)
#             elevation = np.arctan2(z, np.sqrt(x**2 + y**2))

#             azimuth_index = int(azimuth / resolution[0])
#             elevation_index = int(elevation / resolution[1])

#             if (azimuth_index, elevation_index) not in grid:
#                 grid[(azimuth_index, elevation_index)] = []

#             grid[(azimuth_index, elevation_index)].append(point)

#         downsampled_points = []
#         for key, points in grid.items():
#             if points:
#                 avg_point = np.mean(points, axis=0)
#                 downsampled_points.append(avg_point)

#         return downsampled_points

# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarDownsamplingNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

