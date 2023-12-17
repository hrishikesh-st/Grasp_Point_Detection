#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import numpy as np
from sklearn.cluster import DBSCAN

class PointCloudClustering(Node):
    def __init__(self):
        super().__init__('Clustering_Node')

        self.pc_subscriber = self.create_subscription(
            PointCloud2,
            '/processed_pointcloud_data',
            self.cluster_pointcloud_cb,
            10
        )
        self.pc_subscriber
        self.clustered_pc_publisher= self.create_publisher(PointCloud2, "/clustered_point_cloud", 10)

    def cluster_pointcloud_cb(self, data):
        """
        Callback function for clustering pointcloud
        This function performs clustering on the pointcloud using DBSCAN and publishes
        the clustered pointcloud

        Args:
            msg (sensor_msgs.msg.PointCloud2): Point cloud

        Returns:
            None
        """

        points_in_cam_frame = point_cloud2.read_points_numpy(data)
        point_coords=points_in_cam_frame[:, :3]

        # Find the optimal value for eps and min_samples
        k_n=10

        # Perform clustering using DBSCAN
        dbscan = DBSCAN(eps=0.008, min_samples=k_n)  # You might need to adjust eps and min_samples
        dbscan_clusters = dbscan.fit_predict(point_coords)

        # Assign intensity values based on cluster labels
        intensity_values = np.where(dbscan_clusters == 0, 80, dbscan_clusters)
        intensity_values = np.where(intensity_values == 1, 160, intensity_values)
        intensity_values = np.where(intensity_values == 2, 240, intensity_values)

        # Create a fourth column with intensity values for non-outlier points
        unfiltered_point_cloud_with_intensity = np.column_stack((point_coords, intensity_values))

        # Remove outliers
        point_cloud_with_intensity=unfiltered_point_cloud_with_intensity[dbscan_clusters != -1]

        # Publish the clustered pointcloud
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]

        header = Header()
        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()

        pc2 = point_cloud2.create_cloud(header, fields, point_cloud_with_intensity)
        self.clustered_pc_publisher.publish(pc2)

def main(args=None):
    rclpy.init(args=args)
    Clustering_Node = PointCloudClustering()
    rclpy.spin(Clustering_Node)
    Clustering_Node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()