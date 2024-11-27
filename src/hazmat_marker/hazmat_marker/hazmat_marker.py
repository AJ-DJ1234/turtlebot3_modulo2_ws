import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import tf_transformations

class HazmatMarkerNode(Node):
    def __init__(self):
        super().__init__('hazmat_marker_node')
        self.marker_pub = self.create_publisher(MarkerArray, '/hazmat_markers', 10)
        self.hazmat_sub = self.create_subscription(String, '/hazmat_publisher', self.hazmat_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_position = None
        self.hazmat_detected = False
        self.hazmat_name = "No detections"

    def hazmat_callback(self, msg):
        if msg.data != "No detections":
            self.hazmat_detected = True
            self.hazmat_name = msg.data
        else:
            self.hazmat_detected = False

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose
        if self.hazmat_detected:
            self.add_marker()

    def add_marker(self):
        marker_array = MarkerArray()
        
        # Text Marker for Hazmat Name
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = self.current_position.position.x
        text_marker.pose.position.y = self.current_position.position.y
        text_marker.pose.position.z = 1.0
        text_marker.pose.orientation = self.current_position.orientation
        text_marker.scale.z = 0.2  # Reduced text size
        text_marker.color.r = 1.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        text_marker.text = self.hazmat_name
        text_marker.id = self.get_clock().now().to_msg().sec  # Unique ID
        
        marker_array.markers.append(text_marker)
        
        # Sphere Marker at Detection Point
        sphere_marker = Marker()
        sphere_marker.header.frame_id = "map"
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        sphere_marker.pose.position.x = self.current_position.position.x
        sphere_marker.pose.position.y = self.current_position.position.y
        sphere_marker.pose.position.z = 0.0  # Ground level
        sphere_marker.pose.orientation.w = 1.0  # Neutral orientation
        
        # Adjust the size of the sphere
        sphere_marker.scale.x = 0.1  # Diameter in meters
        sphere_marker.scale.y = 0.1
        sphere_marker.scale.z = 0.1
        
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 1.0
        sphere_marker.color.b = 0.0
        sphere_marker.color.a = 1.0
        sphere_marker.id = self.get_clock().now().to_msg().sec + 1  # Unique ID
        
        marker_array.markers.append(sphere_marker)
        
        # Publish markers
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = HazmatMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
