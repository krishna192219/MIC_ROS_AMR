#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster 


class ScanSerialNode(Node):

    def __init__(self):

        super().__init__('scan_serial_node')

        # =====================================================
        # SERIAL
        # =====================================================

        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 115200

        self.ser = serial.Serial(
            self.serial_port,
            self.baud_rate,
            timeout=0.01
        )

        self.get_logger().info(
            f'Opened {self.serial_port} at '
            f'{self.baud_rate} baud'
        )

        # =====================================================
        # ROBOT POSE
        # =====================================================

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.heading = 0.0

        self.pose_received = False

        # =====================================================
        # SENSOR
        # =====================================================

        # VL53 is 6 cm in front of robot center
        self.sensor_offset = 0.060

        # =====================================================
        # PUBLISHERS
        # =====================================================

        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/scan_points',
            10
        )

        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        # =====================================================
        # TF
        # =====================================================

        self.tf_broadcaster = TransformBroadcaster(self)

        # =====================================================
        # STATIC TF: BASE_LINK -> LASER
        # =====================================================
        # VL53L0X is mounted 6 cm in front of the robot center.
        # This publishes the transform permanently on /tf_static
        # while this node is running.
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        laser_transform = TransformStamped()
        laser_transform.header.stamp = self.get_clock().now().to_msg()
        laser_transform.header.frame_id = 'base_link'
        laser_transform.child_frame_id = 'laser'

        laser_transform.transform.translation.x = self.sensor_offset
        laser_transform.transform.translation.y = 0.0
        laser_transform.transform.translation.z = 0.0

        laser_transform.transform.rotation.x = 0.0
        laser_transform.transform.rotation.y = 0.0
        laser_transform.transform.rotation.z = 0.0
        laser_transform.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(laser_transform)

        self.get_logger().info(
            'Published static TF: base_link -> laser '
            f'(x={self.sensor_offset:.3f} m)'
        )

        # =====================================================
        # SCAN
        # =====================================================

        self.scan_points = []
        self.scanning = False

        # =====================================================
        # LASER SCAN
        # =====================================================

        self.scan_start_heading = 0.0

        self.scan_ranges = [float('inf')] * 360

        self.scan_angle_min = -math.pi
        self.scan_angle_max = math.pi
        self.scan_angle_increment = math.radians(1.0)

        self.scan_min_range = 0.03
        self.scan_max_range = 2.0

        # =====================================================
        # SERIAL TIMER
        # =====================================================

        self.serial_timer = self.create_timer(
            0.005,
            self.read_serial
        )

        # =====================================================
        # TF TIMER
        # =====================================================

        self.tf_timer = self.create_timer(
            0.05,
            self.publish_tf
        )

        self.get_logger().info(
            'Waiting for POSE / SCAN data...'
        )

    # =========================================================
    # SERIAL
    # =========================================================

    def read_serial(self):

        try:

            # Process all currently available serial lines
            # so the ROS node does not fall behind.

            while self.ser.in_waiting > 0:

                line = self.ser.readline().decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                if not line:
                    continue

                # =================================================
                # POSE
                # =================================================

                if line.startswith('POSE,'):

                    parts = line.split(',')

                    if len(parts) != 4:
                        continue

                    try:

                        self.robot_x = float(parts[1])
                        self.robot_y = float(parts[2])
                        self.heading = float(parts[3])

                        self.pose_received = True

                    except ValueError:

                        continue

                    continue

                # =================================================
                # SCAN START
                # =================================================

                if line == 'SCAN_START':

                    self.scan_points = []

                    self.scan_ranges = [float('inf')] * 360

                    self.scan_start_heading = self.heading

                    self.scanning = True

                    self.get_logger().info(
                        '360° scan started'
                    )

                    continue

                # =================================================
                # SCAN END
                # =================================================

                if line == 'SCAN_END':

                    self.scanning = False

                    self.get_logger().info(
                        f'Scan complete: '
                        f'{len(self.scan_points)} points'
                    )

                    self.publish_cloud()
                    self.publish_laserscan()

                    continue

                # =================================================
                # SCAN DATA
                # =================================================

                if line.startswith('SCAN,'):

                    parts = line.split(',')

                    if len(parts) != 5:
                        continue

                    try:

                        robot_x = float(parts[1])
                        robot_y = float(parts[2])
                        heading = float(parts[3])
                        distance_mm = float(parts[4])

                    except ValueError:

                        continue

                    distance = distance_mm / 1000.0

                    # Ignore invalid measurements

                    if distance <= self.scan_min_range:
                        continue

                    if distance > self.scan_max_range:
                        continue

                    # ------------------------------------------------
                    # VL53 POSITION
                    # ------------------------------------------------

                    sensor_x = (
                        robot_x
                        + self.sensor_offset
                        * math.cos(heading)
                    )

                    sensor_y = (
                        robot_y
                        + self.sensor_offset
                        * math.sin(heading)
                    )

                    # ------------------------------------------------
                    # OBSTACLE POINT
                    # ------------------------------------------------

                    point_x = (
                        sensor_x
                        + distance
                        * math.cos(heading)
                    )

                    point_y = (
                        sensor_y
                        + distance
                        * math.sin(heading)
                    )

                    point_z = 0.0

                    # ------------------------------------------------
                    # SAVE SCAN DATA
                    # ------------------------------------------------

                    if self.scanning:

                        self.scan_points.append(
                            (
                                point_x,
                                point_y,
                                point_z
                            )
                        )

                        # ---------------------------------------------
                        # Convert robot heading to scan angle
                        # relative to heading at SCAN_START.
                        # ---------------------------------------------

                        relative_angle = (
                            heading - self.scan_start_heading
                        )

                        relative_angle = math.atan2(
                            math.sin(relative_angle),
                            math.cos(relative_angle)
                        )

                        # ---------------------------------------------
                        # Convert angle to 1-degree bin
                        # ---------------------------------------------

                        index = int(
                            round(
                                (relative_angle - self.scan_angle_min)
                                / self.scan_angle_increment
                            )
                        )

                        index %= 360

                        # ---------------------------------------------
                        # Keep closest measurement in each bin
                        # ---------------------------------------------

                        if distance < self.scan_ranges[index]:

                            self.scan_ranges[index] = distance

        except Exception as e:

            self.get_logger().error(
                f'Serial error: {e}'
            )

    # =========================================================
    # CONTINUOUS TF
    # =========================================================

    def publish_tf(self):

        if not self.pose_received:
            return

        self.publish_odom_tf(
            self.robot_x,
            self.robot_y,
            self.heading
        )

    # =========================================================
    # ODOM -> BASE_LINK TF
    # =========================================================

    def publish_odom_tf(
        self,
        x,
        y,
        yaw
    ):

        transform = TransformStamped()

        transform.header.stamp = (
            self.get_clock().now().to_msg()
        )

        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0

        transform.transform.rotation.z = math.sin(
            yaw / 2.0
        )

        transform.transform.rotation.w = math.cos(
            yaw / 2.0
        )

        self.tf_broadcaster.sendTransform(
            transform
        )

    # =========================================================
    # POINT CLOUD
    # =========================================================

    def publish_cloud(self):

        if not self.scan_points:

            self.get_logger().warning(
                'No scan points collected!'
            )

            return

        header = Header()

        header.stamp = (
            self.get_clock().now().to_msg()
        )

        header.frame_id = 'odom'

        cloud = point_cloud2.create_cloud_xyz32(
            header,
            self.scan_points
        )

        self.pointcloud_pub.publish(cloud)

        self.get_logger().info(
            f'Published {len(self.scan_points)} '
            f'points to /scan_points'
        )

    # =========================================================
    # LASER SCAN
    # =========================================================

    def publish_laserscan(self):

        scan = LaserScan()

        scan.header.stamp = (
            self.get_clock().now().to_msg()
        )

        scan.header.frame_id = 'laser'

        scan.angle_min = self.scan_angle_min
        scan.angle_max = self.scan_angle_max
        scan.angle_increment = self.scan_angle_increment

        scan.time_increment = 0.0
        scan.scan_time = 0.0

        scan.range_min = self.scan_min_range
        scan.range_max = self.scan_max_range

        scan.ranges = self.scan_ranges

        self.scan_pub.publish(scan)

        valid_points = sum(
            1
            for r in self.scan_ranges
            if math.isfinite(r)
        )

        self.get_logger().info(
            f'Published LaserScan: '
            f'{valid_points} valid ranges to /scan'
        )


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = ScanSerialNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()