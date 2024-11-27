# joint_state_publisher_custom/joint_state_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import threading

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher_custom')

        # Create subscribers for motor positions
        self.motor1_sub = self.create_subscription(
            Float32,
            '/motor1_position',
            self.motor1_callback,
            10
        )
        self.motor2_sub = self.create_subscription(
            Float32,
            '/motor2_position',
            self.motor2_callback,
            10
        )

        # Initialize joint state message
        self.joint_state = JointState()
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.name = ['joint_motor_1_rotor', 'joint_motor_2_rotor']
        self.joint_state.position = [0.0, 0.0]
        self.joint_state.velocity = []
        self.joint_state.effort = []

        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Timer to publish joint states at 50 Hz
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Mutex for thread-safe operations
        self.lock = threading.Lock()

        self.get_logger().info('Joint State Publisher Node has been started.')

    def motor1_callback(self, msg):
        with self.lock:
            self.joint_state.position[0] = msg.data
            self.get_logger().debug(f'Updated joint_motor_1_rotor position to {msg.data} rad')

    def motor2_callback(self, msg):
        with self.lock:
            self.joint_state.position[1] = msg.data
            self.get_logger().debug(f'Updated joint_motor_2_rotor position to {msg.data} rad')

    def timer_callback(self):
        with self.lock:
            self.joint_state.header.stamp = self.get_clock().now().to_msg()
            self.joint_pub.publish(self.joint_state)
            self.get_logger().debug('Published joint states.')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Joint State Publisher Node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
