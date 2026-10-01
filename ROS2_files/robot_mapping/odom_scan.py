import math
import serial

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class SerialScanNode(Node):

    def __init__(self):

        super().__init__("serial_scan_node")

        # =================================================
        # SERIAL
        # =================================================

        self.port = "/dev/ttyUSB0"
        self.baud = 115200

        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=0.01
        )

        self.get_logger().info(
            f"Connected to {self.port}"
        )

        # =================================================
        # ROS PUBLISHERS
        # =================================================

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            10
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            "/tof_points",
            10
        )

        # =================================================
        # TF
        # =================================================

        self.tf_broadcaster = TransformBroadcaster(self)

        self.static_tf_broadcaster = (
            StaticTransformBroadcaster(self)
        )

        self.publish_laser_tf()

        # =================================================
        # ROBOT STATE
        # =================================================

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.heading = 0.0

        # =================================================
        # ACCUMULATED POINTS
        # =================================================

        self.points = []

        # Maximum number of stored points
        self.max_points = 10000

        # =================================================
        # TIMER
        # =================================================

        self.timer = self.create_timer(
            0.005,
            self.read_serial
        )

        self.get_logger().info(
            "Serial -> Odometry + TF + ToF PointCloud"
        )

    # =====================================================
    # QUATERNION
    # =====================================================

    def quaternion_from_yaw(self, yaw):

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        return 0.0, 0.0, qz, qw

    # =====================================================
    # STATIC TF
    # base_link -> laser
    # =====================================================

    def publish_laser_tf(self):

        transform = TransformStamped()

        transform.header.stamp = (
            self.get_clock().now().to_msg()
        )

        transform.header.frame_id = "base_link"
        transform.child_frame_id = "laser"

        # VL53 position relative to robot
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        # Sensor points forward along +X
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(
            transform
        )

    # =====================================================
    # SERIAL
    # =====================================================

    def read_serial(self):

        while self.ser.in_waiting:

            try:

                line = self.ser.readline().decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

                if not line:
                    continue

                # Only process SCAN messages
                if not line.startswith("SCAN,"):
                    continue

                data = line.split(",")

                if len(data) != 5:
                    continue

                # =================================================
                # ESP32 DATA
                # =================================================

                self.robot_x = float(data[1])
                self.robot_y = float(data[2])
                self.heading = float(data[3])

                distance_mm = float(data[4])

                if distance_mm <= 0:
                    continue

                distance = distance_mm / 1000.0

                # Sensor limits
                if distance < 0.03:
                    continue

                if distance > 1.2:
                    continue

                # =================================================
                # CURRENT TIME
                # =================================================

                stamp = self.get_clock().now().to_msg()

                # =================================================
                # ODOM
                # =================================================

                self.publish_odom(stamp)

                # =================================================
                # TF
                # =================================================

                self.publish_odom_tf(stamp)

                # =================================================
                # CALCULATE GLOBAL TOF POINT
                # =================================================

                point_x = (
                    self.robot_x
                    + distance * math.cos(self.heading)
                )

                point_y = (
                    self.robot_y
                    + distance * math.sin(self.heading)
                )

                point_z = 0.0

                # =================================================
                # STORE POINT
                # =================================================

                self.points.append(
                    (point_x, point_y, point_z)
                )

                # Prevent unlimited memory growth
                if len(self.points) > self.max_points:

                    self.points.pop(0)

                # =================================================
                # PUBLISH POINT CLOUD
                # =================================================

                self.publish_point_cloud(stamp)

            except ValueError:

                continue

    # =====================================================
    # ODOMETRY
    # =====================================================

    def publish_odom(self, stamp):

        msg = Odometry()

        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.robot_x
        msg.pose.pose.position.y = self.robot_y
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = (
            self.quaternion_from_yaw(
                self.heading
            )
        )

        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        self.odom_pub.publish(msg)

    # =====================================================
    # ODOM -> BASE_LINK
    # =====================================================

    def publish_odom_tf(self, stamp):

        transform = TransformStamped()

        transform.header.stamp = stamp

        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = self.robot_x
        transform.transform.translation.y = self.robot_y
        transform.transform.translation.z = 0.0

        qx, qy, qz, qw = (
            self.quaternion_from_yaw(
                self.heading
            )
        )

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(
            transform
        )

    # =====================================================
    # POINT CLOUD
    # =====================================================

    def publish_point_cloud(self, stamp):

        header = self.get_clock().now().to_msg()

        from std_msgs.msg import Header

        cloud_header = Header()

        cloud_header.stamp = stamp

        # VERY IMPORTANT:
        # Points are already expressed in odom coordinates
        cloud_header.frame_id = "odom"

        cloud = point_cloud2.create_cloud_xyz32(
            cloud_header,
            self.points
        )

        self.cloud_pub.publish(cloud)

    # =====================================================
    # CLEANUP
    # =====================================================

    def destroy_node(self):

        if hasattr(self, "ser"):

            self.ser.close()

        super().destroy_node()


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = SerialScanNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()