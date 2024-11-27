import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import tf_transformations
import math

class HazmatMarkerNode(Node):
    def __init__(self):
        super().__init__('hazmat_marker_node_al')
        marker_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/hazmat_markers', marker_qos_profile)
        self.hazmat_sub = self.create_subscription(String, '/hazmat_publisher', self.hazmat_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        lidar_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, lidar_qos_profile)
        self.current_position = None
        self.current_orientation = None
        self.lidar_ranges = []
        self.hazmat_detected = False
        self.hazmat_name = "No detections"
        self.detection_angle = None
        self.marker_added = False

        self.detected_hazmat = []

    def hazmat_callback(self, msg):
        self.get_logger().info(f"Hazmat callback received: {msg.data}")
        if msg.data != "No detections":
            self.hazmat_detected = True
            self.hazmat_name = msg.data
            self.marker_added = False
        else:
            self.hazmat_detected = False
            self.marker_added = False

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
        self.get_logger().info(f"Odometry updated: {self.current_position}, {self.current_orientation}")

    def lidar_callback(self, msg):
        self.lidar_ranges = msg.ranges
        self.get_logger().info(f"LIDAR data received with {len(self.lidar_ranges)} ranges")
        if self.hazmat_detected and not self.marker_added:
            self.detection_angle = (msg.angle_min + msg.angle_max) / 2.0
            self.add_marker()
            self.marker_added = True

    def add_marker(self):
        if self.current_position is None or self.current_orientation is None or not self.lidar_ranges:
            self.get_logger().warn("Missing data for marker placement")
            return

        _, _, yaw = tf_transformations.euler_from_quaternion([
            self.current_orientation.x,
            self.current_orientation.y,
            self.current_orientation.z,
            self.current_orientation.w
        ])

        detection_distance = min(self.lidar_ranges)  
        hazmat_x = self.current_position.x + detection_distance * math.cos(yaw + self.detection_angle + math.pi)
        hazmat_y = self.current_position.y + detection_distance * math.sin(yaw + self.detection_angle + math.pi)

        # Verificar si el HAZMAT ya fue marcado cerca
        for hazmat in self.detected_hazmat:
            existing_x, existing_y, existing_name = hazmat
            distance = math.sqrt((hazmat_x - existing_x) ** 2 + (hazmat_y - existing_y) ** 2)
            if distance < 0.5 and existing_name == self.hazmat_name:                # Una distancia menos a 0.5 metros y del mismo tipo
                self.get_logger().info("Hazmat ya marcado, ignorando...")
                return

        # Agregar nuevo marcador HAZMAT
        marker_array = MarkerArray()

        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = hazmat_x
        text_marker.pose.position.y = hazmat_y
        text_marker.pose.position.z = 0.0
        text_marker.pose.orientation.w = 1.0  
        text_marker.scale.z = 0.1
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        text_marker.text = self.hazmat_name
        text_marker.id = self.get_clock().now().to_msg().sec  

        marker_array.markers.append(text_marker)

        sphere_marker = Marker()
        sphere_marker.header.frame_id = "map"
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        sphere_marker.pose.position.x = hazmat_x
        sphere_marker.pose.position.y = hazmat_y
        sphere_marker.pose.position.z = 0.0  
        sphere_marker.pose.orientation.w = 1.0  

        sphere_marker.scale.x = 0.1  
        sphere_marker.scale.y = 0.1
        sphere_marker.scale.z = 0.1

        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 1.0
        sphere_marker.color.a = 1.0
        sphere_marker.id = self.get_clock().now().to_msg().sec + 1  

        marker_array.markers.append(sphere_marker)

        self.get_logger().info(f"Publishing marker at position: {hazmat_x}, {hazmat_y}")
        self.marker_pub.publish(marker_array)

        # Agregar a la lista de HAZMATS detectados
        self.detected_hazmat.append((hazmat_x, hazmat_y, self.hazmat_name))

def main(args=None):
    rclpy.init(args=args)
    node = HazmatMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
