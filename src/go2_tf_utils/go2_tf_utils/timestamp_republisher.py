#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2


class PointCloudTimestampUpdater(Node):

    def __init__(self):
        super().__init__('pointcloud_timestamp_updater')

        # Subscriber
        self.sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',   # change as needed
            self.callback,
            10
        )

        # Publisher
        self.pub = self.create_publisher(
            PointCloud2,
            '/livox/cloud',  # change as needed
            10
        )

    def callback(self, msg: PointCloud2):
        # Create a copy of the incoming message
        new_msg = PointCloud2()

        # Copy all fields
        new_msg.header = msg.header
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.fields = msg.fields
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = msg.point_step
        new_msg.row_step = msg.row_step
        new_msg.data = msg.data
        new_msg.is_dense = msg.is_dense

        # Overwrite timestamp with current time
        new_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish updated message
        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTimestampUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()