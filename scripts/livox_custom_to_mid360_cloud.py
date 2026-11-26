#!/usr/bin/env python3

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
    PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
    PointField(name="timestamp", offset=20, datatype=PointField.FLOAT64, count=1),
]


class LivoxCloudConverter(Node):
    def __init__(self):
        super().__init__("livox_custom_to_mid360_cloud")
        self.publisher = self.create_publisher(PointCloud2, "/livox/lidar_pc2", 10)
        self.subscription = self.create_subscription(
            CustomMsg, "/livox/lidar", self.convert, 10
        )

    def convert(self, msg):
        points = (
            (
                point.x,
                point.y,
                point.z,
                float(point.reflectivity),
                point.tag,
                point.line,
                float(msg.timebase + point.offset_time),
            )
            for point in msg.points
        )
        cloud = point_cloud2.create_cloud(msg.header, FIELDS, points)
        cloud.is_dense = True
        self.publisher.publish(cloud)


def main():
    rclpy.init()
    node = LivoxCloudConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
