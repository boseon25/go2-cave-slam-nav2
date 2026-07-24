#!/usr/bin/env python3
"""Publish odom->base_link TF and /odom from Gazebo ground-truth,
bypassing champ_base's leg-kinematics odometry (which diverges to
huge garbage values in this go2 port).

The raw ground-truth pose faithfully mirrors the quadruped's actual
physical body motion, including the small step-to-step bob/sway that
is normal for a walking gait (unlike a wheeled robot, which rolls
smoothly and is usually further smoothed by an EKF). Left unfiltered,
that high-frequency wobble propagates through the whole TF chain out
to the lidar frame, making walls/scan points appear to jitter with
the robot in RViz. A light exponential moving average on position and
orientation filters that out while still tracking real motion.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class GroundTruthOdomBridge(Node):
    def __init__(self):
        super().__init__('ground_truth_odom_bridge')
        self.declare_parameter('position_alpha', 0.3)
        self.declare_parameter('orientation_alpha', 0.3)
        self.pos_alpha = self.get_parameter('position_alpha').value
        self.rot_alpha = self.get_parameter('orientation_alpha').value

        self.br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.sub = self.create_subscription(
            Odometry, 'odom/ground_truth', self.cb, 10)

        self._filtered_pos = None
        self._filtered_quat = None

    @staticmethod
    def _lerp(a, b, alpha):
        return a + alpha * (b - a)

    def _filter_position(self, p):
        if self._filtered_pos is None:
            self._filtered_pos = [p.x, p.y, p.z]
        else:
            a = self.pos_alpha
            self._filtered_pos[0] = self._lerp(self._filtered_pos[0], p.x, a)
            self._filtered_pos[1] = self._lerp(self._filtered_pos[1], p.y, a)
            self._filtered_pos[2] = self._lerp(self._filtered_pos[2], p.z, a)
        return self._filtered_pos

    def _filter_orientation(self, q):
        new = [q.x, q.y, q.z, q.w]
        if self._filtered_quat is None:
            self._filtered_quat = new
        else:
            old = self._filtered_quat
            # keep on the same hemisphere so lerp takes the short path
            if sum(o * n for o, n in zip(old, new)) < 0.0:
                new = [-c for c in new]
            a = self.rot_alpha
            blended = [self._lerp(o, n, a) for o, n in zip(old, new)]
            norm = math.sqrt(sum(c * c for c in blended)) or 1.0
            self._filtered_quat = [c / norm for c in blended]
        return self._filtered_quat

    def cb(self, msg: Odometry):
        px, py, pz = self._filter_position(msg.pose.pose.position)
        qx, qy, qz, qw = self._filter_orientation(msg.pose.pose.orientation)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = px
        t.transform.translation.y = py
        t.transform.translation.z = pz
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.br.sendTransform(t)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = px
        out.pose.pose.position.y = py
        out.pose.pose.position.z = pz
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist
        self.odom_pub.publish(out)


def main():
    rclpy.init()
    node = GroundTruthOdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
