#!/usr/bin/env python3
"""Publish a deterministic synthetic /initialpose from reference map odometry."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def normalize_quaternion(q):
    norm = math.sqrt(sum(value * value for value in q))
    return tuple(value / norm for value in q)


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


class SyntheticInitialPose(Node):
    def __init__(self):
        super().__init__("synthetic_initial_pose")
        self.declare_parameter("delay_seconds", 1.0)
        self.declare_parameter("translation_x", 0.0)
        self.declare_parameter("translation_y", 0.0)
        self.declare_parameter("translation_z", 0.0)
        self.declare_parameter("yaw_degrees", 0.0)
        self.declare_parameter("position_sigma", 0.25)
        self.declare_parameter("yaw_sigma_degrees", 5.0)
        self.declare_parameter("roll_pitch_sigma_degrees", 2.0)
        self.declare_parameter("map_frame_id", "map")

        self.delay_seconds = float(self.get_parameter("delay_seconds").value)
        self.translation = (
            float(self.get_parameter("translation_x").value),
            float(self.get_parameter("translation_y").value),
            float(self.get_parameter("translation_z").value),
        )
        self.yaw_degrees = float(self.get_parameter("yaw_degrees").value)
        self.position_sigma = float(self.get_parameter("position_sigma").value)
        self.yaw_sigma_degrees = float(
            self.get_parameter("yaw_sigma_degrees").value
        )
        self.roll_pitch_sigma_degrees = float(
            self.get_parameter("roll_pitch_sigma_degrees").value
        )
        self.map_frame = self.get_parameter("map_frame_id").value
        self.first_timestamp = None
        self.published = False

        qos = QoSProfile(depth=100)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/benchmark/initialpose", 1
        )
        self.create_subscription(
            Odometry,
            "/reference/odometry_map",
            self.reference_callback,
            qos,
        )

    def reference_callback(self, reference):
        if self.published:
            return
        timestamp = stamp_seconds(reference.header.stamp)
        if self.first_timestamp is None:
            self.first_timestamp = timestamp
        if timestamp - self.first_timestamp < self.delay_seconds:
            return

        message = PoseWithCovarianceStamped()
        message.header.stamp = reference.header.stamp
        message.header.frame_id = self.map_frame
        message.pose.pose = reference.pose.pose
        message.pose.pose.position.x += self.translation[0]
        message.pose.pose.position.y += self.translation[1]
        message.pose.pose.position.z += self.translation[2]

        half_yaw = math.radians(self.yaw_degrees) * 0.5
        yaw_offset = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
        original = (
            reference.pose.pose.orientation.x,
            reference.pose.pose.orientation.y,
            reference.pose.pose.orientation.z,
            reference.pose.pose.orientation.w,
        )
        perturbed = quaternion_multiply(yaw_offset, original)
        (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        ) = perturbed

        position_variance = self.position_sigma ** 2
        roll_pitch_variance = math.radians(
            self.roll_pitch_sigma_degrees
        ) ** 2
        yaw_variance = math.radians(self.yaw_sigma_degrees) ** 2
        message.pose.covariance[0] = position_variance
        message.pose.covariance[7] = position_variance
        message.pose.covariance[14] = position_variance
        message.pose.covariance[21] = roll_pitch_variance
        message.pose.covariance[28] = roll_pitch_variance
        message.pose.covariance[35] = yaw_variance

        self.publisher.publish(message)
        self.published = True
        self.get_logger().info(
            "Published synthetic initial pose at %.6f with offset "
            "(%.3f, %.3f, %.3f) m, %.3f deg yaw"
            % (
                timestamp,
                self.translation[0],
                self.translation[1],
                self.translation[2],
                self.yaw_degrees,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
