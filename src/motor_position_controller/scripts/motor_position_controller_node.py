#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MotorPositionControllerNode(Node):
    def __init__(self):
        super().__init__('motor_position_controller')
        # Create publishers for motor1 and motor2 positions
        self.motor1_pub = self.create_publisher(Float32, '/motor1_position', 10)
        self.motor2_pub = self.create_publisher(Float32, '/motor2_position', 10)
        
        # Timer to periodically publish positions
        timer_period = 0.05  # seconds (20 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize positions
        self.motor1_position = 0.0
        self.motor2_position = 0.0
        
        # Define position limits
        self.MAX_POSITION = 12.56  # radians (example: π rad)
        self.MIN_POSITION = -12.56  # radians (example: -π rad)
        
        # Log info
        self.get_logger().info('Motor Position Controller Node has been started.')
        
        # Start a separate thread for keyboard input
        import threading
        thread = threading.Thread(target=self.keyboard_input_thread)
        thread.daemon = True
        thread.start()

    def timer_callback(self):
        # Publish the current positions
        msg1 = Float32()
        msg1.data = self.motor1_position
        self.motor1_pub.publish(msg1)
        
        msg2 = Float32()
        msg2.data = self.motor2_position
        self.motor2_pub.publish(msg2)
        
        self.get_logger().debug(f'Published Motor1 Position: {self.motor1_position} rad')
        self.get_logger().debug(f'Published Motor2 Position: {self.motor2_position} rad')

    def keyboard_input_thread(self):
        self.get_logger().info('Control Instructions:')
        self.get_logger().info('  To set both motors, type "T <value>", e.g., "T 3.14"')
        self.get_logger().info('  To set Motor1, type "T1 <value>", e.g., "T1 1.57"')
        self.get_logger().info('  To set Motor2, type "T2 <value>", e.g., "T2 2.00"')
        self.get_logger().info('  Press "q" to quit.')
        
        while True:
            try:
                cmd = input().strip()
                
                if cmd.startswith('T '):
                    # Set both motors to the same position
                    try:
                        value = float(cmd[2:])
                        # Apply position limits
                        value = max(min(value, self.MAX_POSITION), self.MIN_POSITION)
                        self.motor1_position = value
                        self.motor2_position = value
                        self.get_logger().info(f'Set both Motor1 and Motor2 positions to {value:.2f} rad')
                    except ValueError:
                        self.get_logger().error('Invalid position value. Use "T <number>", e.g., "T 3.14"')
                
                elif cmd.startswith('T1 '):
                    # Set Motor1 to a specific position
                    try:
                        value = float(cmd[3:])
                        # Apply position limits
                        value = max(min(value, self.MAX_POSITION), self.MIN_POSITION)
                        self.motor1_position = value
                        self.get_logger().info(f'Set Motor1 position to {value:.2f} rad')
                    except ValueError:
                        self.get_logger().error('Invalid position value. Use "T1 <number>", e.g., "T1 1.57"')
                
                elif cmd.startswith('T2 '):
                    # Set Motor2 to a specific position
                    try:
                        value = float(cmd[3:])
                        # Apply position limits
                        value = max(min(value, self.MAX_POSITION), self.MIN_POSITION)
                        self.motor2_position = value
                        self.get_logger().info(f'Set Motor2 position to {value:.2f} rad')
                    except ValueError:
                        self.get_logger().error('Invalid position value. Use "T2 <number>", e.g., "T2 2.00"')
                
                elif cmd == 'q':
                    self.get_logger().info('Quitting Motor Position Controller Node.')
                    rclpy.shutdown()
                    break
                
                else:
                    self.get_logger().warn('Invalid command. Use "T <number>", "T1 <number>", "T2 <number>", or "q".')
            
            except EOFError:
                # Handle end-of-file (e.g., Ctrl+D)
                self.get_logger().info('EOF detected. Quitting Motor Position Controller Node.')
                rclpy.shutdown()
                break
            except KeyboardInterrupt:
                # Handle keyboard interrupt (e.g., Ctrl+C)
                self.get_logger().info('KeyboardInterrupt detected. Quitting Motor Position Controller Node.')
                rclpy.shutdown()
                break

def main(args=None):
    rclpy.init(args=args)
    node = MotorPositionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
