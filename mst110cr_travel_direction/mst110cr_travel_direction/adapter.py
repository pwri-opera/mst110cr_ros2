"""Relay each Twist once, optionally correcting its travel direction."""

from copy import deepcopy
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from mst110cr_travel_direction.policy import DirectionPolicy


class TravelDirectionAdapter(Node):
    def __init__(self):
        super().__init__('travel_direction_adapter')

        def setting(name, default):
            return self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value

        input_topic = setting('input_topic', 'cmd_vel_smoothed')
        output_topic = setting('output_topic', 'cmd_vel')
        joint_topic = setting('joint_states_topic', 'joint_states')
        self.joint_name = setting('joint_name', 'rotate_joint')
        self.invert_angular_z = setting('invert_angular_z', False)
        self.policy = DirectionPolicy(
            enabled=setting('enable_correction', False),
            timeout=setting('joint_state_timeout', 0.5),
            threshold_deg=setting('reverse_threshold_deg', 90.0),
            stop_band_deg=setting('stop_band_deg', 0.0),
        )
        if self.resolve_topic_name(input_topic) == self.resolve_topic_name(output_topic):
            raise ValueError('input_topic and output_topic must differ (including remappings)')
        self.output = self.create_publisher(Twist, output_topic, 1)
        self.status = self.create_publisher(String, '~/state', 1)
        self.joints = self.create_subscription(
            JointState, joint_topic, self.on_joints, qos_profile_sensor_data)
        self.commands = self.create_subscription(Twist, input_topic, self.on_command, 1)
        # Diagnostics only: never replay velocity commands from a timer.
        self.status_timer = self.create_timer(0.2, self.publish_status)
        self.get_logger().info(
            f'{self.resolve_topic_name(input_topic)} -> '
            f'{self.resolve_topic_name(output_topic)}; '
            f'correction={self.policy.enabled}')

    def on_joints(self, message):
        # JointState arrays need not have a fixed joint ordering.
        try:
            index = message.name.index(self.joint_name)
        except ValueError:
            # Other publishers may send partial JointStates; do not refresh age.
            return
        angle = message.position[index] if index < len(message.position) else None
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.policy.update(angle, time.monotonic(), stamp)

    def current_state(self):
        return self.policy.state(time.monotonic(), self.get_clock().now().nanoseconds * 1e-9)

    def publish_status(self):
        self.status.publish(String(data=self.current_state()))

    def on_command(self, message):
        state = self.current_state()
        if state == 'passthrough':
            self.output.publish(message)
            return
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if state not in ('forward', 'reverse') or not all(map(math.isfinite, values)):
            self.output.publish(Twist())
            return
        result = deepcopy(message)
        if state == 'reverse':
            result.linear.x *= -1.0
            if self.invert_angular_z:
                result.angular.z *= -1.0
        self.output.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TravelDirectionAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
