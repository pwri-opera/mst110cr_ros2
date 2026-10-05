"""Initialize EKF from the first GNSS pose, then release its sensor inputs."""
from copy import deepcopy
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import quaternion_matrix, quaternion_multiply


def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def valid_pose(message):
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, *message.pose.covariance]
    return (bool(message.header.frame_id) and stamp_ns(message) > 0
            and all(math.isfinite(v) for v in values)
            and sum(v*v for v in [q.x, q.y, q.z, q.w]) > 1e-12
            and all(message.pose.covariance[i] > 0 for i in [0, 7, 35]))


def initial_pose(message, transform, target_frame):
    """Transform position, orientation and covariance, then constrain to 2D."""
    result = deepcopy(message)
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    t = transform.transform.translation
    r = transform.transform.rotation
    rotation = [r.x, r.y, r.z, r.w]
    matrix = quaternion_matrix(rotation)[:3, :3]
    xyz = matrix @ np.array([p.x, p.y, p.z]) + [t.x, t.y, t.z]
    quat = quaternion_multiply(rotation, [q.x, q.y, q.z, q.w])
    quat /= np.linalg.norm(quat)
    x, y, z, w = quat
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    result.header.frame_id = target_frame
    result.pose.pose.position.x = float(xyz[0])
    result.pose.pose.position.y = float(xyz[1])
    result.pose.pose.position.z = 0.0
    result.pose.pose.orientation.x = 0.0
    result.pose.pose.orientation.y = 0.0
    result.pose.pose.orientation.z = math.sin(yaw/2)
    result.pose.pose.orientation.w = math.cos(yaw/2)
    jacobian = np.zeros((6, 6))
    jacobian[:3, :3] = matrix
    jacobian[3:, 3:] = matrix
    covariance = jacobian @ np.array(message.pose.covariance).reshape(6, 6) @ jacobian.T
    for i in [2, 3, 4]:
        covariance[i, :] = 0.0
        covariance[:, i] = 0.0
        covariance[i, i] = 1e-6
    result.pose.covariance = covariance.ravel().tolist()
    return result


class GnssEkfInitializer(Node):
    def __init__(self):
        super().__init__('gnss_ekf_initializer')
        self.declare_parameter('world_frame', 'map')
        self.target = self.get_parameter('world_frame').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, 'ekf/initial_pose', 1)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'ekf/gnss_pose', 10)
        self.odom_pub = self.create_publisher(Odometry, 'ekf/odom_pose', 10)
        self.create_subscription(PoseWithCovarianceStamped, 'gnss_pose', self.pose_callback, 10)
        self.create_subscription(Odometry, 'odom_pose', self.odom_callback, 100)
        self.create_subscription(Odometry, 'odometry/global', self.ekf_callback, 10)
        self.pending = None
        self.sent = False
        self.ready = False
        self.initial_stamp = 0
        self.timer = self.create_timer(0.05, self.initialize)
        self.get_logger().info('Waiting for the first GNSS pose before releasing EKF inputs')

    def pose_callback(self, message):
        if not valid_pose(message):
            return
        if self.ready:
            if stamp_ns(message) > self.initial_stamp:
                self.pose_pub.publish(message)
        elif self.pending is None:
            self.pending = deepcopy(message)

    def odom_callback(self, message):
        if self.ready and stamp_ns(message) > self.initial_stamp:
            self.odom_pub.publish(message)

    def ekf_callback(self, message):
        if self.sent and not self.ready and stamp_ns(message) >= self.initial_stamp:
            self.ready = True
            self.timer.cancel()
            self.get_logger().info('EKF initialized from first GNSS x/y/yaw; releasing inputs')

    def initialize(self):
        if self.sent or self.pending is None:
            return
        # A dedicated one-shot x/y/yaw measurement initializes all three state
        # variables even when continuous GNSS yaw fusion is disabled. Waiting
        # for EKF output before releasing odom prevents an initialization race.
        if not self.initial_pub.get_subscription_count():
            return
        try:
            transform = self.buffer.lookup_transform(
                self.target, self.pending.header.frame_id,
                Time.from_msg(self.pending.header.stamp))
        except TransformException:
            return
        message = initial_pose(self.pending, transform, self.target)
        self.initial_stamp = stamp_ns(self.pending)
        self.initial_pub.publish(message)
        self.sent = True


def main(args=None):
    rclpy.init(args=args)
    node = GnssEkfInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
