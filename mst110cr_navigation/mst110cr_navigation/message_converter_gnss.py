#!/usr/bin/env python3
"""Add configurable fallback covariance to GNSS PoseStamped messages.

The input reference frame, timestamp and antenna pose are preserved. This node
performs no map conversion, heading correction or antenna-to-base compensation.
The defaults are conservative standalone-GNSS assumptions, not receiver accuracy
measurements: sigma_xy=5 m, sigma_z=10 m. Orientation is untrusted by default;
the accompanying EKF configuration only fuses x/y.
"""
import math
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node


def covariance_diagonal(position_variance_m2, orientation_variance_rad2):
    """Build a ROS pose covariance in x, y, z, roll, pitch, yaw order."""
    for values in (position_variance_m2, orientation_variance_rad2):
        if len(values) != 3 or any(not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError('Each variance parameter must contain three finite positive values.')
    covariance = [0.0] * 36
    for i, variance in enumerate(list(position_variance_m2) + list(orientation_variance_rad2)):
        covariance[i * 7] = float(variance)
    return covariance


def convert_pose(message, covariance):
    """Preserve the incoming pose and header; only add covariance."""
    output = PoseWithCovarianceStamped()
    output.header = deepcopy(message.header)
    output.pose.pose = deepcopy(message.pose)
    output.pose.covariance = covariance
    return output


class GnssPoseConverter(Node):
    def __init__(self):
        super().__init__('message_converter_gnss')
        self.declare_parameter('input_topic', 'global_pose')
        self.declare_parameter('output_topic', 'gnss_pose')
        self.declare_parameter('position_variance_m2', [25.0, 25.0, 100.0])
        self.declare_parameter('orientation_variance_rad2', [math.pi ** 2] * 3)
        self.covariance = covariance_diagonal(
            self.get_parameter('position_variance_m2').value,
            self.get_parameter('orientation_variance_rad2').value,
        )
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)
        self.subscription = self.create_subscription(PoseStamped, input_topic, self.callback, 10)
        self.get_logger().info(
            f'{input_topic} (PoseStamped) -> {output_topic} (PoseWithCovarianceStamped); '
            'preserving frame and stamp; using configured fallback covariance'
        )

    def callback(self, message):
        self.publisher.publish(convert_pose(message, self.covariance))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GnssPoseConverter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
