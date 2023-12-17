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

import math
from shapely.ops import cascaded_union, polygonize
from shapely.geometry import MultiPoint, MultiLineString, GeometryCollection, MultiPolygon
from scipy.spatial import Delaunay


class GraspSynthesis(Node):
    """ Grasp synthesis node

    Args:

    Attributes:
        processed_pointcloud_subscriber (rclpy.Subscriber): Subscribe to processed point cloud
        thresh_pc_publisher (rclpy.Publisher): Publish thresholded point cloud
        gp_publisher (rclpy.Publisher): Publish grasping points marker
        threshold (list): Threshold for point cloud (lower and upper limit)
        grasping_points (visualization_msgs.msg.Marker): Marker for grasping points

    Methods:
        processed_pointcloud_cb(self, msg): Callback function for processed point cloud
        _init_marker(self, marker, points): Initialize marker
    """

    def __init__(self):
        super().__init__('grasp_points_systhesis')

        self.processed_pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/clustered_point_cloud',
            self.processed_pointcloud_cb,
            10
        )
        self.processed_pointcloud_subscriber
        self.thresh_pc_publisher1= self.create_publisher(PointCloud2, "/thresholded_pointcloud1", 10)
        self.gp_publisher1= self.create_publisher(Marker, "/grasping_points1", 10)
        self.thresh_pc_publisher2= self.create_publisher(PointCloud2, "/thresholded_pointcloud2", 10)
        self.gp_publisher2= self.create_publisher(Marker, "/grasping_points2", 10)
        self.thresh_pc_publisher3= self.create_publisher(PointCloud2, "/thresholded_pointcloud3s", 10)
        self.gp_publisher3= self.create_publisher(Marker, "/grasping_points3", 10)
        self.publishers_ = [
            [self.thresh_pc_publisher1, self.gp_publisher1],
            [self.thresh_pc_publisher2, self.gp_publisher2],
            [self.thresh_pc_publisher3, self.gp_publisher3]
        ]

        # Threshold for point cloud (lower and upper limit)
        self.threshold = [0.011, 0.018] 

        # Grasping points are visualized in RViz using visualization_msgs.msg.Marker
        # the SPHERE_LIST marker type is used as it can visualize multiple points in the single marker message
        self.grasping_points = Marker()
        self.grasping_point1 = Point()
        self.grasping_point2 = Point()

    def _init_marker(self, marker, points):
        """ Initialize and populate Marker message fields

        Args:
            marker (visualization_msgs.msg.Marker): Marker
            points (list): List of points

        Returns:
            None
        """

        marker.header.frame_id = "world" # Marker Frame
        marker.type = marker.SPHERE_LIST # Marker Type
        marker.action = marker.ADD

        # Marker size
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        # Marker color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Marker positions and orientation
        marker.pose.orientation.w = 1.0
        marker.points = points

    def processed_pointcloud_cb(self, msg):
        """ Callback function for processed point cloud
        This function performs thresholding on processed pointcloud to get the top surface
        of the object and then calculates the grasping points on the object based on the
        grasping heuristic.

        Args:
            msg (sensor_msgs.msg.PointCloud2): Point cloud

        Returns:
            None
        """

        processed_pointcloud = point_cloud2.read_points_numpy(msg)

        cluster_dict = {}
        for point in processed_pointcloud:
            intensity = point[-1]
            if intensity not in cluster_dict:
                cluster_dict[intensity] = []
            cluster_dict[intensity].append(point)

        # all points of the point cloud are arranged in the ascending order of z coordinates
        publisher_index = 0
        for key in cluster_dict:
            _coords_arr = np.array(cluster_dict[key])
            _coords = _coords_arr[_coords_arr[:, 2].argsort()]
            # _coords = processed_pointcloud[processed_pointcloud[:, 2].argsort()]

            # Thresholding point cloud to get the top surface of the object (layer of points)
            # All points below the lower limit are discarded
            lower_limit = _coords[:, 2][-1] - self.threshold[1]
            i = np.argwhere(_coords[:, 2] > lower_limit)
            thresh_coords1 = _coords[i].squeeze()

            flatten_coords = thresh_coords1[:, :2]

            def add_edge(edges, edge_points, coords, i, j):
                """
                Add a line between the i-th and j-th points,
                if not in the list already
                """
                if (i, j) in edges or (j, i) in edges:
                    # already added
                    return
                edges.add( (i, j) )
                edge_points.append(coords[ [i, j] ])

            def alpha_shape(points, alpha):
                """
                Compute the alpha shape (concave hull) of a set
                of points.
                @param points: np.array of shape (n,2) points.
                @param alpha: alpha value to influence the
                gooeyness of the border. Smaller numbers
                don't fall inward as much as larger numbers.
                Too large, and you lose everything!
                """
                if len(points) < 4:
                    # When you have a triangle, there is no sense
                    # in computing an alpha shape.
                    return MultiPoint(list(points)).convex_hull

                coords = np.array([point for point in points])
                tri = Delaunay(coords)
                edges = set()
                edge_points = []
                for simplex in tri.simplices:
                    ia, ib, ic = simplex  # Indices of the vertices of the triangle
                    pa = coords[ia]
                    pb = coords[ib]
                    pc = coords[ic]

                    a = math.sqrt((pa[0] - pb[0])**2 + (pa[1] - pb[1])**2)
                    b = math.sqrt((pb[0] - pc[0])**2 + (pb[1] - pc[1])**2)
                    c = math.sqrt((pc[0] - pa[0])**2 + (pc[1] - pa[1])**2)
                    s = (a + b + c) / 2.0
                    area = math.sqrt(s * (s - a) * (s - b) * (s - c))
                    circum_r = a * b * c / (4.0 * area)

                    if circum_r < 1.0 / alpha:
                        add_edge(edges, edge_points, coords, ia, ib)
                        add_edge(edges, edge_points, coords, ib, ic)
                        add_edge(edges, edge_points, coords, ic, ia)

                m = MultiLineString(edge_points)
                triangles = list(polygonize(m))
                return cascaded_union(triangles), edge_points

            # Alpha shape parameter
            alpha = 300
            alpha_shape_result = alpha_shape(flatten_coords, alpha) # Alpha shape result
            alpha_shape_polygon, _ = alpha_shape_result # Alpha shape polygon
            exterior_coords = np.array(alpha_shape_polygon.exterior.coords) # Exterior coordinates of alpha shape polygon
            exterior_coords_zeros = np.hstack((exterior_coords, np.zeros((exterior_coords.shape[0], 1)))) # Exterior coordinates with z = 0

            flatten_coords = exterior_coords

            # Function to find the normal at each point
            k = 3 # Number of neighbors to calculate normal
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

            # calculate center of thresholded point cloud
            _center = [np.mean(flatten_coords[:, 0]), np.mean(flatten_coords[:, 1])]

            # calculate points for grasping
            min_dist = float('inf')
            _point1 = None # 2d
            normal1 = None

            # find point closest to center, and its corresponding normal (Point 1)
            for point, normal in zip(flatten_coords, normal_vectors):
                _dist = np.sqrt((point[0] - _center[0])**2 + (point[1] - _center[1])**2)
                if _dist < min_dist:
                    min_dist = _dist
                    _point1 = point.copy()
                    normal1 = normal.copy()

            # Find point which satisfies the grasping heuristic (Point 2)
            # The angles between the vector connecting the 2 grasping points and the corresponding normals
            # should be between 0 to 10 or 170 to 180 degrees
            # Both these limits are considered because the directions of the normals can be flipped
            _point2 = None # 2d
            for point, normal in zip(flatten_coords, normal_vectors):
                _dist = np.sqrt((point[0] - _point1[0])**2 + (point[1] - _point1[1])**2)
                if _dist == 0: continue # skip if point is same as point1
                # unit vector between point and point1
                _vec = np.array([(point[0]-_point1[0])/_dist, (point[1]-_point1[1])/_dist])

                _deg1 = np.degrees(np.arccos(np.dot(_vec, normal1))) # Angle between vector and normal1 (point1), degrees
                _deg2 = np.degrees(np.arccos(np.dot(_vec, normal))) # Angle between vector and normal (point), degrees

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


            for point in thresh_coords1:
                if point[0] == _point1[0] and point[1] == _point1[1]:
                    point1 = point.copy()
                if point[0] == _point2[0] and point[1] == _point2[1]:
                    point2 = point.copy()


            # Converting points 1 and 2 to geometry_msgs.msg.Point
            self.grasping_point1.x = float(point1[0])
            self.grasping_point1.y = float(point1[1])
            self.grasping_point1.z = float(point1[2])

            self.grasping_point2.x = float(point2[0])
            self.grasping_point2.y = float(point2[1])
            self.grasping_point2.z = float(point2[2])

            _grasping_points = [self.grasping_point1, self.grasping_point2] # List of grasping points

            # initialize grasping_points Marker message with list of grasping points
            self._init_marker(self.grasping_points, _grasping_points)

            fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

            header = Header()
            header.frame_id = "world"
            header.stamp = self.get_clock().now().to_msg()

            pc2 = point_cloud2.create_cloud(header, fields, exterior_coords_zeros)
            self.publishers_[publisher_index][0].publish(pc2) # publish thresholded point cloud
            self.publishers_[publisher_index][1].publish(self.grasping_points)
            publisher_index += 1


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = GraspSynthesis()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()