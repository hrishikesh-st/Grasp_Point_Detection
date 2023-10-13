#!/usr/bin/env python3

import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import Header
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs_py import point_cloud2
import numpy as np
from sklearn.decomposition import PCA


class GraspSynthesis(Node):
    def __init__(self):
        super().__init__('grasp_points_systhesis')

        self.processed_pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/processed_pointcloud_data',
            self.processed_pointcloud_cb,
            10
        )
        self.processed_pointcloud_subscriber
        self.pc_publisher= self.create_publisher(PointCloud2, "/thresholded_pointcloud", 10)
        self.gp_publisher= self.create_publisher(Marker, "/grasping_points", 10)

        self.threshold = [0.012, 0.018] # hammer
        # self.threshold = [0.005, 0.01] # drill, coke

        self.grasping_points = Marker()
        self.grasping_point1 = Point()
        self.grasping_point2 = Point()
        self.id = 0

    def _init_marker(self, marker, points):
        marker.header.frame_id = "world"
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.points = points

    def processed_pointcloud_cb(self, msg):

        processed_pointcloud = point_cloud2.read_points_numpy(msg)

        _coords = processed_pointcloud[processed_pointcloud[:, 2].argsort()] # Ascending order
        
        lower_limit = _coords[:, 2][-1] - self.threshold[1]
        i = np.argwhere(_coords[:, 2] > lower_limit)
        thresh_coords1 = _coords[i].squeeze()

        upper_limit = thresh_coords1[:, 2][-1] - self.threshold[0]
        i = np.argwhere(thresh_coords1[:, 2] < upper_limit)
        thresh_coords2 = thresh_coords1[i].squeeze()

        # Flattened array
        flatten_coords = thresh_coords2[:, :2]
        zeros_column = np.zeros((flatten_coords.shape[0], 1))
        modified_falttened_coords = np.hstack((flatten_coords, zeros_column))

        # np.save('hammer.npy', flatten_coords)
        # print("Saved numpy file")

        # calculate normals
        # Define the number of neighbors to consider (k)
        k = 3

        # Function to find the normal at each point
        def calculate_normals(point_cloud, k):
            normals = []
            for point in point_cloud:
                # Calculate distances to all other points
                distances = np.linalg.norm(point_cloud - point, axis=1)
                # Sort by distance and get indices of the k nearest neighbors
                nearest_indices = np.argsort(distances)[1:k+1]
                # Get the neighboring points
                neighbors = point_cloud[nearest_indices]
                # Use PCA to calculate the normal vector
                pca = PCA(n_components=2)
                pca.fit(neighbors)
                normal = pca.components_[1]
                normals.append(normal)
            return normals

        # Calculate normals for each point
        normal_vectors = calculate_normals(flatten_coords, k)
        _center = [np.mean(flatten_coords[:, 0]), np.mean(flatten_coords[:, 1])]

        # calculate points for grasping
        min_dist = float('inf')
        _point1 = None # 2d
        normal1 = None

        for point, normal in zip(flatten_coords, normal_vectors):
            _dist = np.sqrt((point[0] - _center[0])**2 + (point[1] - _center[1])**2)
            if _dist < min_dist: 
                min_dist = _dist
                _point1 = point.copy()
                normal1 = normal.copy()
        
        _point2 = None # 2d
        for point, normal in zip(flatten_coords, normal_vectors):
            _dist = np.sqrt((point[0] - _point1[0])**2 + (point[1] - _point1[1])**2)
            if _dist == 0: continue
            _vec = np.array([(point[0]-_point1[0])/_dist, (point[1]-_point1[1])/_dist]) # unit vector

            _deg1 = np.degrees(np.arccos(np.dot(_vec, normal1)))
            _deg2 = np.degrees(np.arccos(np.dot(_vec, normal)))

            flag1 = False
            flag2 = False

            if _deg1 < 10 or _deg1 > 170:
                flag1 = True
            if _deg2 < 10 or _deg2 > 170:
                flag2 = True

            if flag1 and flag2:
                _point2 = point.copy()

        point1 = [0.0, 0.0, 0.0]
        point2 = [0.0, 0.0, 0.0]

        for point in thresh_coords2:
            if point[0] == _point1[0] and point[1] == _point1[1]:
                point1 = point.copy()
            if point[0] == _point2[0] and point[1] == _point2[1]:
                point2 = point.copy()

        self.grasping_point1.x = float(point1[0])
        self.grasping_point1.y = float(point1[1])
        self.grasping_point1.z = float(point1[2])

        self.grasping_point2.x = float(point2[0])
        self.grasping_point2.y = float(point2[1])
        self.grasping_point2.z = float(point2[2])

        _grasping_points = [self.grasping_point1, self.grasping_point2]

        self._init_marker(self.grasping_points, _grasping_points)

        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

        header = Header()
        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()
    
        pc2 = point_cloud2.create_cloud(header, fields, thresh_coords2)
        self.pc_publisher.publish(pc2)
        self.gp_publisher.publish(self.grasping_points)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = GraspSynthesis()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()