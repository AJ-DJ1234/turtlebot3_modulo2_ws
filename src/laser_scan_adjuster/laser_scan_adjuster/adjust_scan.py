import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data

class LaserScanAdjuster(Node):
    def __init__(self):
        super().__init__('laser_scan_adjuster')

        # Use the QoS profile specifically for sensor data
        qos_profile = qos_profile_sensor_data

        # Subscribe to the /scan topic with the sensor data QoS profile
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Replace with your actual scan topic if different
            self.scan_callback,
            qos_profile  # Apply the sensor data QoS profile
        )
        self.publisher = self.create_publisher(LaserScan, '/adjusted_scan', qos_profile)
        self.expected_size = 227  # Set this to the expected number of range readings

    def scan_callback(self, msg):
        current_size = len(msg.ranges)

        if current_size < self.expected_size:
            # Pad with the last range value
            padding = [msg.ranges[-1]] * (self.expected_size - current_size)
            msg.ranges.extend(padding)
            self.get_logger().info(f'Padded ranges from {current_size} to {self.expected_size}')
        elif current_size > self.expected_size:
            # Truncate the list
            msg.ranges = msg.ranges[:self.expected_size]
            self.get_logger().info(f'Truncated ranges from {current_size} to {self.expected_size}')

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanAdjuster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
