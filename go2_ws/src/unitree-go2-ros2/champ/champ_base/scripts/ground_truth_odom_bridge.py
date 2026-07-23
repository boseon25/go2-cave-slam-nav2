#!/usr/bin/env python3
"""Publish odom->base_link TF and /odom from Gazebo ground-truth,
bypassing champ_base's leg-kinematics odometry (which diverges to
huge garbage values in this go2 port)."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class GroundTruthOdomBridge(Node):
    def __init__(self):
        super().__init__('ground_truth_odom_bridge')
        self.br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.sub = self.create_subscription(
            Odometry, 'odom/ground_truth', self.cb, 10)

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.pose = msg.pose
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
