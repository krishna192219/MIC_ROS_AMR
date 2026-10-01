import math
import serial

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class SerialScanNode(Node):

    def __init__(self):

        super().__init__("serial_scan_node")

        # =================================================
        # SERIAL
        # =================================================

        self.port = "/dev/ttyUSB0"
        self.baud = 115200

        try:

            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0.01
            )

            self.get_logger().info(
                f"Connected to {self.port}"
            )

        except serial.SerialException as e:

            self.get_logger().error(
                f"Serial error: {e}"
            )

            raise

        # =================================================
        # SCAN PARAMETERS
        # =================================================

        self.angle_min = -math.pi
        self.angle_max = math.pi

        # 1 degree resolution

        self.angle_increment = math.radians(1.0)

        self.num_bins = 360

        # =================================================
        # ACCUMULATED SCAN
        # =================================================

        self.ranges = [
            float("inf")
            for _ in range(self.num_bins)
        ]

        # =================================================
        # ROS PUBLISHER
        # =================================================

        self.scan_pub = self.create_publisher(
            LaserScan,
            "/scan",
            10
        )

        # =================================================
        # TIMER
        # =================================================

        self.timer = self.create_timer(
            0.005,
            self.read_serial
        )

        # Publish accumulated scan periodically

        self.publish_timer = self.create_timer(
            0.05,
            self.publish_scan
        )

        self.get_logger().info(
            "Serial → accumulated LaserScan started"
        )

    # =====================================================
    # READ SERIAL
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

                # -----------------------------------------
                # Only SCAN packets
                # -----------------------------------------

                if not line.startswith("SCAN,"):
                    continue

                data = line.split(",")

                if len(data) != 5:
                    continue

                # -----------------------------------------
                # ESP32 DATA
                # -----------------------------------------

                robot_x = float(data[1])
                robot_y = float(data[2])
                heading = float(data[3])
                distance_mm = float(data[4])

                if distance_mm <= 0:
                    continue

                distance_m = distance_mm / 1000.0

                # -----------------------------------------
                # IGNORE OUT-OF-RANGE VALUES
                # -----------------------------------------

                if distance_m < 0.03:
                    continue

                if distance_m > 1.2:
                    continue

                # -----------------------------------------
                # NORMALIZE HEADING
                #
                # ESP32 heading is 0 → 2π
                #
                # Convert to -π → +π
                # -----------------------------------------

                angle = heading

                if angle > math.pi:
                    angle -= 2.0 * math.pi

                # -----------------------------------------
                # CONVERT ANGLE TO BIN
                # -----------------------------------------

                bin_index = int(
                    round(
                        (angle - self.angle_min)
                        / self.angle_increment
                    )
                )

                # -----------------------------------------
                # CHECK BIN
                # -----------------------------------------

                if 0 <= bin_index < self.num_bins:

                    # Keep the closest measurement
                    # if multiple readings hit same bin.

                    if distance_m < self.ranges[bin_index]:

                        self.ranges[bin_index] = distance_m

            except ValueError:

                continue

    # =====================================================
    # PUBLISH LASER SCAN
    # =====================================================

    def publish_scan(self):

        msg = LaserScan()

        # -----------------------------------------------
        # HEADER
        # -----------------------------------------------

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = "laser"

        # -----------------------------------------------
        # ANGLES
        # -----------------------------------------------

        msg.angle_min = self.angle_min

        msg.angle_max = self.angle_max

        msg.angle_increment = self.angle_increment

        # -----------------------------------------------
        # TIMING
        # -----------------------------------------------

        msg.time_increment = 0.0

        msg.scan_time = 0.05

        # -----------------------------------------------
        # RANGE
        # -----------------------------------------------

        msg.range_min = 0.03

        msg.range_max = 1.2

        # -----------------------------------------------
        # DATA
        # -----------------------------------------------

        msg.ranges = self.ranges

        self.scan_pub.publish(msg)

    # =====================================================
    # CLEANUP
    # =====================================================

    def destroy_node(self):

        if hasattr(self, "ser"):

            self.ser.close()

        super().destroy_node()


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