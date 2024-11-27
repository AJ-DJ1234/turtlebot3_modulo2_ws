#!/usr/bin/env python3
"""
POI Manager Node for TurtleBot3.
Allows adding, removing, listing, and commanding navigation to Points of Interest (POIs).
Publishes visualization markers for POIs in RViz.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav2_simple_commander.robot_navigator import BasicNavigator
from visualization_msgs.msg import Marker, MarkerArray
import math
import sys


class POI:
    """Class representing a Point of Interest."""
    def __init__(self, name, x, y, radius):
        self.name = name
        self.position = Point(x=x, y=y, z=0.0)
        self.radius = radius

class POIManager(Node):
    def __init__(self):
        super().__init__('poi_manager')

        # Initialize BasicNavigator
        self.navigator = BasicNavigator()
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is ready for use!")

                # Publisher for POI markers
        self.marker_pub = self.create_publisher(MarkerArray, 'poi_markers', 10)

        # Timer to periodically publish markers
        self.marker_timer = self.create_timer(1.0, self.publish_markers)


        # POI storage
        self.pois = {}  # Dictionary to store POIs by name

        # Add POIs
        self.pois['kitchen'] = POI(
            'kitchen',
            -3.644530773162842,
            -2.4650981426239014,
            0.5
        )
        self.get_logger().info("POIs have been added.")

        # Check for command-line argument
        if len(sys.argv) > 1:
            poi_name = sys.argv[1]
            if poi_name in self.pois:
                self.navigate_to_poi(poi_name)
            else:
                self.get_logger().warn(f"POI '{poi_name}' does not exist.")
        else:
            self.get_logger().info("No POI name provided. Please run the node with a POI name argument.")
            self.get_logger().info("Example: ros2 run turtlebot3_poi_navigation poi_manager.py kitchen")

    def navigate_to_poi(self, poi_name):
        """Navigates to the specified POI."""
        if poi_name in self.pois:
            poi = self.pois[poi_name]
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position = poi.position
            goal_pose.pose.orientation = self.yaw_to_quaternion(0.0)

            self.navigator.goToPose(goal_pose)
            self.get_logger().info(f"Navigating to POI: {poi_name} at ({poi.position.x}, {poi.position.y})")
        else:
            self.get_logger().warn(f"POI '{poi_name}' not found.")

    def publish_markers(self):
        """Publishes visualization markers for all POIs."""
        marker_array = MarkerArray()
        for idx, (name, poi) in enumerate(self.pois.items()):
            # Sphere Marker for POI position
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'poi_markers'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = poi.position
            marker.scale.x = poi.radius * 2  # Diameter
            marker.scale.y = poi.radius * 2
            marker.scale.z = 0.1
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

            # Text Marker for POI label
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'poi_labels'
            text_marker.id = idx + 1000  # Ensure unique IDs
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position = poi.position
            text_marker.pose.position.z += 0.2  # Slightly above the sphere
            text_marker.scale.z = 0.3  # Text height
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = name
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


    def yaw_to_quaternion(self, yaw):
        """Converts a yaw angle to a quaternion."""
        from geometry_msgs.msg import Quaternion
        half_yaw = yaw * 0.5
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half_yaw)
        q.w = math.cos(half_yaw)
        return q

def main(args=None):
    rclpy.init(args=args)
    poi_manager = POIManager()
    rclpy.spin(poi_manager)

    # Shutdown
    poi_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
