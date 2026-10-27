"""Initialize EKF from GNSS position and lower-body yaw, then release inputs."""
from copy import deepcopy
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
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


def lower_heading_pose(message, now_ns):
    """Convert the COM3 world-referenced lower-body orientation to a yaw pose.

    This publisher uses Imu.header.frame_id as the orientation reference frame,
    not an IMU mounting frame, so it must not be fed to EKF as a normal imu0.
    Reject unavailable orientation and messages older than 0.5 seconds.
    """
    q = message.orientation
    values = [q.x, q.y, q.z, q.w]
    variance = message.orientation_covariance[8]
    age = (now_ns - stamp_ns(message)) / 1e9
    if (message.header.frame_id != 'world' or stamp_ns(message) <= 0
            or not -0.1 <= age <= 0.5
            or message.orientation_covariance[0] < 0
            or not math.isfinite(variance) or variance <= 0
            or not all(math.isfinite(v) for v in values)):
        return None
    norm = math.sqrt(sum(v*v for v in values))
    if norm < 1e-6:
        return None
    x, y, z, w = [v / norm for v in values]
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    result = PoseWithCovarianceStamped()
    result.header = deepcopy(message.header)
    result.pose.pose.orientation.z = math.sin(yaw / 2)
    result.pose.pose.orientation.w = math.cos(yaw / 2)
    # EKF pose2 selects yaw only: position/roll/pitch are not measurements.
    result.pose.covariance[35] = variance
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
        self.heading_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'ekf/lower_heading', 10)
        self.create_subscription(Imu, 'imu', self.heading_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'gnss_pose', self.pose_callback, 10)
        self.create_subscription(Odometry, 'odom_pose', self.odom_callback, 100)
        self.create_subscription(Odometry, 'odometry/global', self.ekf_callback, 10)
        self.pending = None
        self.pending_heading = None
        self.sent = False
        self.ready = False
        self.initial_stamp = 0
        self.last_heading_stamp = 0
        self.timer = self.create_timer(0.05, self.initialize)
        self.get_logger().info('Waiting for GNSS position and lower-body yaw before releasing EKF inputs')

    def pose_callback(self, message):
        if not valid_pose(message):
            return
        if self.ready:
            if stamp_ns(message) > self.initial_stamp:
                self.pose_pub.publish(message)
        elif not self.sent:
            self.pending = deepcopy(message)

    def odom_callback(self, message):
        if self.ready and stamp_ns(message) > self.initial_stamp:
            self.odom_pub.publish(message)

    def heading_callback(self, message):
        # Cache heading for initialization without letting yaw alone start EKF.
        stamp = stamp_ns(message)
        if stamp <= max(self.initial_stamp, self.last_heading_stamp):
            return
        pose = lower_heading_pose(message, self.get_clock().now().nanoseconds)
        if pose is not None:
            if self.ready:
                self.heading_pub.publish(pose)
            elif not self.sent:
                self.pending_heading = deepcopy(message)
            self.last_heading_stamp = stamp

    def ekf_callback(self, message):
        if self.sent and not self.ready and stamp_ns(message) >= self.initial_stamp:
            self.ready = True
            self.timer.cancel()
            self.get_logger().info('EKF initialized from GNSS x/y and lower-body yaw; releasing inputs')

    def initialize(self):
        if self.sent or self.pending is None or self.pending_heading is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        heading = lower_heading_pose(self.pending_heading, now_ns)
        # Keep both measurements fresh while waiting for inputs and TF.
        if (heading is None
                or not -0.1 <= (now_ns - stamp_ns(self.pending)) / 1e9 <= 0.5):
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
            heading_transform = self.buffer.lookup_transform(
                self.target, heading.header.frame_id,
                Time.from_msg(heading.header.stamp))
        except TransformException:
            return
        message = initial_pose(self.pending, transform, self.target)
        heading_in_map = initial_pose(heading, heading_transform, self.target)
        message.pose.pose.orientation = heading_in_map.pose.pose.orientation
        # Position and heading come from independent inputs.
        for i in range(6):
            message.pose.covariance[5 * 6 + i] = 0.0
            message.pose.covariance[i * 6 + 5] = 0.0
        message.pose.covariance[35] = heading_in_map.pose.covariance[35]
        if stamp_ns(heading) > stamp_ns(message):
            message.header.stamp = deepcopy(heading.header.stamp)
        self.initial_stamp = stamp_ns(message)
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
