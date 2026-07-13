#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped

import sensor_msgs_py.point_cloud2 as pc2
import tf_transformations

from transforms3d.quaternions import quat2mat

import numpy as np


class CloudTransformer(Node):

    def __init__(self):
        super().__init__('cloud_transformer')

        # Subscriber
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/utlidar/restamped_cloud',
            self.cloud_callback,
            50
        )

        # Publisher
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            '/utlidar/transformed_cloud',
            50
        )

        # ---------------- TRANSFORM ----------------

        self.body2cloud_trans = TransformStamped()

        self.body2cloud_trans.header.frame_id = "utlidar_frame"
        self.body2cloud_trans.child_frame_id = "utlidar_lidar"

        # Translation
        self.body2cloud_trans.transform.translation.x = 0.0
        self.body2cloud_trans.transform.translation.y = 0.0
        self.body2cloud_trans.transform.translation.z = 0.0

        # Rotation
        quat = tf_transformations.quaternion_from_euler(
            0,
            2.8782,
            0
        )

        self.body2cloud_trans.transform.rotation.x = quat[0]
        self.body2cloud_trans.transform.rotation.y = quat[1]
        self.body2cloud_trans.transform.rotation.z = quat[2]
        self.body2cloud_trans.transform.rotation.w = quat[3]

    # ---------------- CLOUD CALLBACK ----------------

    def cloud_callback(self, data):

        # Convert PointCloud2 -> numpy array
        cloud_arr = pc2.read_points_list(data)
        points = np.array(cloud_arr)

        # Rotation matrix from quaternion
        transform = self.body2cloud_trans.transform

        mat = quat2mat(np.array([
            transform.rotation.w,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z
        ]))

        # Translation vector
        translation = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z
        ])

        # Apply transform
        points[:, 0:3] = points[:, 0:3] @ mat.T + translation

        # Rebuild cloud
        cloud_out = pc2.create_cloud(
            data.header,
            data.fields,
            points.tolist()
        )

        cloud_out.header.frame_id = "utlidar_frame"

        self.cloud_pub.publish(cloud_out)


def main(args=None):

    rclpy.init(args=args)

    node = CloudTransformer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()