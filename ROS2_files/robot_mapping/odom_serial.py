#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


class SerialOdometry(Node):

    def __init__(self):
        super().__init__('serial_odometry') 

        # -----------------------------------------
        # ROBOT PARAMETERS
        # -----------------------------------------

        self.wheel_diameter = 0.044
        self.wheel_separation = 0.095
        self.counts_per_rev = 4096

        self.wheel_circumference = (
            math.pi * self.wheel_diameter
        )

        # -----------------------------------------
        # SERIAL
        # -----------------------------------------

        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.01
            )

            self.get_logger().info(
                f'Connected to {self.serial_port}'
            )

        except serial.SerialException as e:

            self.get_logger().error(
                f'Could not open serial port: {e}'
            )

            raise

        # -----------------------------------------
        # ROS PUBLISHERS
        # -----------------------------------------

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(
            self
        )

        # -----------------------------------------
        # ODOMETRY STATE
        # -----------------------------------------

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left = None
        self.prev_right = None

        # -----------------------------------------
        # TIMER
        # -----------------------------------------

        self.timer = self.create_timer(
            0.01,
            self.update
        )

        self.get_logger().info(
            'Serial odometry started'
        )

    # =================================================
    # SERIAL UPDATE
    # =================================================

    def update(self):

        while self.ser.in_waiting:

            try:
                line = (
                    self.ser.readline()
                    .decode('utf-8')
                    .strip()
                )

            except UnicodeDecodeError:
                continue

            if not line.startswith('ENC,'):
                continue

            parts = line.split(',')

            if len(parts) != 3:
                continue

            try:
                left_count = int(parts[1])
                right_count = int(parts[2])

            except ValueError:
                continue

            self.process_encoder_counts(
                left_count,
                right_count
            )

    # =================================================
    # ODOMETRY
    # =================================================

    def process_encoder_counts(
        self,
        left_count,
        right_count
    ):

        # First encoder message
        if (
            self.prev_left is None or
            self.prev_right is None
        ):

            self.prev_left = left_count
            self.prev_right = right_count

            return

        # -----------------------------------------
        # Detect encoder reset
        # -----------------------------------------
        #
        # Your ESP32 calls resetEncoders() before
        # every 50 cm side and every rotation.
        #
        # Therefore the counts intentionally jump
        # back close to zero.
        #
        # We must NOT interpret that as the robot
        # physically moving backwards 15,000 counts.
        # -----------------------------------------

        if (
            abs(left_count - self.prev_left) > 5000 or
            abs(right_count - self.prev_right) > 5000
        ):

            self.get_logger().info(
                'Encoder reset detected'
            )

            self.prev_left = left_count
            self.prev_right = right_count

            return

        # -----------------------------------------
        # Count change
        # -----------------------------------------

        delta_left = left_count - self.prev_left
        delta_right = right_count - self.prev_right

        self.prev_left = left_count
        self.prev_right = right_count

        # -----------------------------------------
        # Convert counts → wheel distance
        # -----------------------------------------

        left_distance = (
            delta_left /
            self.counts_per_rev
        ) * self.wheel_circumference

        right_distance = (
            delta_right /
            self.counts_per_rev
        ) * self.wheel_circumference

        # -----------------------------------------
        # IMPORTANT:
        #
        # Right encoder is negative during physical
        # forward motion.
        #
        # Convert it so positive right_distance
        # means physical forward movement.
        # -----------------------------------------

        right_distance = -right_distance

        # -----------------------------------------
        # Differential-drive odometry
        # -----------------------------------------

        delta_s = (
            left_distance +
            right_distance
        ) / 2.0

        delta_theta = (
            right_distance -
            left_distance
        ) / self.wheel_separation

        # Midpoint integration
        theta_mid = (
            self.theta +
            delta_theta / 2.0
        )

        self.x += (
            delta_s *
            math.cos(theta_mid)
        )

        self.y += (
            delta_s *
            math.sin(theta_mid)
        )

        self.theta += delta_theta

        # Keep theta bounded
        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        self.publish_odometry(
            left_distance,
            right_distance,
            delta_s,
            delta_theta
        )

    # =================================================
    # QUATERNION
    # =================================================

    def quaternion_from_yaw(self, yaw):

        return (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0)
        )

    # =================================================
    # PUBLISH
    # =================================================

    def publish_odometry(
        self,
        left_distance,
        right_distance,
        delta_s,
        delta_theta
    ):

        now = self.get_clock().now()

        # -----------------------------------------
        # Odometry message
        # -----------------------------------------

        odom = Odometry()

        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        (
            qx,
            qy,
            qz,
            qw
        ) = self.quaternion_from_yaw(
            self.theta
        )

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # -----------------------------------------
        # Publish
        # -----------------------------------------

        self.odom_pub.publish(odom)

        # -----------------------------------------
        # TF
        # -----------------------------------------

        transform = TransformStamped()

        transform.header.stamp = now.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(
            transform
        )


def main(args=None):

    rclpy.init(args=args)

    node = SerialOdometry()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()