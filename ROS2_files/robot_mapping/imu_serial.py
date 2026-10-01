import serial
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu


class IMUSerial(Node):

    def __init__(self):
        super().__init__('imu_serial')

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.publish_imu_tf()

        self.ser = serial.Serial(
            '/dev/ttyUSB0',
            115200,
            timeout=0.1
        )

        self.imu_pub = self.create_publisher(
            Imu,
            '/imu/data_raw',
            10
        )

        self.timer = self.create_timer(
            0.01,       # 100 Hz
            self.read_serial
        )

        self.get_logger().info('IMU serial node started')

    def read_serial(self):

        try:
            line = self.ser.readline().decode(
                'utf-8',
                errors='ignore'
            ).strip()

            if not line.startswith('IMU,'):
                return

            data = line.split(',')

            if len(data) != 7:
                return

            ax = float(data[1])
            ay = float(data[2])
            az = float(data[3])

            gx = float(data[4])
            gy = float(data[5])
            gz = float(data[6])

            msg = Imu()

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # Acceleration: g -> m/s^2
            msg.linear_acceleration.x = ax * 9.80665
            msg.linear_acceleration.y = ay * 9.80665
            msg.linear_acceleration.z = az * 9.80665

            # Angular velocity: deg/s -> rad/s
            msg.angular_velocity.x = math.radians(gx)
            msg.angular_velocity.y = math.radians(gy)
            msg.angular_velocity.z = math.radians(gz)

            # We do NOT have orientation yet.
            msg.orientation_covariance[0] = -1.0

            # Example covariance values.
            # We can tune these later for robot_localization.
            msg.angular_velocity_covariance[0] = 0.01
            msg.angular_velocity_covariance[4] = 0.01
            msg.angular_velocity_covariance[8] = 0.01

            msg.linear_acceleration_covariance[0] = 0.1
            msg.linear_acceleration_covariance[4] = 0.1
            msg.linear_acceleration_covariance[8] = 0.1

            self.imu_pub.publish(msg)

        except (ValueError, serial.SerialException):
            pass
    def publish_imu_tf(self):

        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()

        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'imu_link'

        # IMU is approximately at the robot center
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        # Identity rotation for now
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):

    rclpy.init(args=args)

    node = IMUSerial()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()