#!/usr/bin/env python3

import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import Header
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import numpy as np
from tf2_ros import TransformListener, Buffer
import tf_transformations
# from tf2_ros.TansformListener import TransformListener


class PointCloudTransformation(Node):
    """ Point cloud transformation node

    Attributes:
        pc_subscriber (rclpy.Subscriber): Subscribe to raw point cloud
        transformed_pc_publisher (rclpy.Publisher): Publish transformed point cloud

    Methods:
        pointcloud_cb(self, data): Callback function for point cloud
    """

    def __init__(self):
        super().__init__('image_subscriber')

        self.pc_subscriber = self.create_subscription(
            PointCloud2,
            '/realsense/points',
            self.pointcloud_cb,
            10
        )

        self.subscription_pointcloud
        self.pc_publisher= self.create_publisher(PointCloud2, "/transformed_pointcloud_data", 10)
        self.tf_buffer = Buffer()
        self.tf_transformer = TransformListener(self.tf_buffer, self)


    def transformed_pointcloud_cb(self, data):
        """ Callback function for point cloud

        Args:
            data (sensor_msgs.msg.PointCloud2): Point cloud

        Returns:
            None
        """

        points_in_cam_frame = point_cloud2.read_points_numpy(data) # 307200 x 4

        # Separate xyz points and intensity
        _cam_data = points_in_cam_frame[:, :3]
        _cam_int = points_in_cam_frame[:, 3]

        # Replace intensity values with 1 to perform transformation
        _cam_data = np.hstack((_cam_data, np.ones_like(_cam_int).reshape(-1, 1)))
        t = self.tf_buffer.lookup_transform('camera_link_optical', 'world', rclpy.time.Time())
        
        #Convert to Homogeneous transformation matrix
        tf_rot = tf_transformations.quaternion_matrix([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
        tf_rot[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]


        # Transforming points to world frame
        _world_data = np.matmul(tf_rot, _cam_data.T)
        points_in_world_frame = np.hstack((_world_data.T[:, :3], _cam_int.reshape(-1, 1)))

        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]

        header = Header()
        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()

        pc2 = point_cloud2.create_cloud(header, fields, points_in_world_frame)

        self.pc_publisher.publish(pc2)
        

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = PointCloudTransformation()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()


