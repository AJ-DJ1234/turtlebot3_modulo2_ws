#!/usr/bin/env python3
"""
POI Manager Node for TurtleBot3.
Allows adding, removing, listing, and commanding navigation to Points of Interest (POIs).
Publishes visualization markers for POIs in RViz.
Includes persistent storage and a simple GUI using Tkinter.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav2_simple_commander.robot_navigator import BasicNavigator
from visualization_msgs.msg import Marker, MarkerArray
import math
import sys
import yaml
import os
import threading
import tkinter as tk
from tkinter import messagebox

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

        # Path to the POI file
        self.poi_file = os.path.expanduser('~/pois.yaml')

        # Load POIs from file
        self.load_pois(self.poi_file)

    def load_pois(self, filepath):
        """Loads POIs from a YAML file."""
        if not os.path.exists(filepath):
            self.get_logger().info(f"POI file {filepath} does not exist. Starting with no POIs.")
            return

        try:
            with open(filepath, 'r') as file:
                data = yaml.safe_load(file)
                if data is not None:
                    for name, attrs in data.items():
                        self.pois[name] = POI(name, attrs['x'], attrs['y'], attrs['radius'])
                    self.get_logger().info(f"Loaded POIs from {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to load POIs: {str(e)}")

    def save_pois(self, filepath):
        """Saves POIs to a YAML file."""
        try:
            with open(filepath, 'w') as file:
                data = {name: {'x': poi.position.x, 'y': poi.position.y, 'radius': poi.radius}
                        for name, poi in self.pois.items()}
                yaml.dump(data, file)
            self.get_logger().info(f"POIs saved to {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to save POIs: {str(e)}")

    def start_gui(self):
        """Initializes and runs the Tkinter GUI."""
        self.root = tk.Tk()
        self.root.title("POI Manager")

        # Create the GUI components
        self.create_widgets()

        # Start the Tkinter main loop
        self.root.mainloop()

    def create_widgets(self):
        """Creates the GUI widgets."""
        # Listbox to display POIs
        self.poi_listbox = tk.Listbox(self.root, height=10, width=50)
        self.poi_listbox.pack()

        # Buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack()

        self.add_button = tk.Button(button_frame, text="Add POI", command=self.add_poi_gui)
        self.add_button.pack(side=tk.LEFT)

        self.delete_button = tk.Button(button_frame, text="Delete POI", command=self.delete_poi_gui)
        self.delete_button.pack(side=tk.LEFT)

        self.navigate_button = tk.Button(button_frame, text="Navigate to POI", command=self.navigate_to_poi_gui)
        self.navigate_button.pack(side=tk.LEFT)

        # Populate the listbox with existing POIs
        self.update_poi_listbox()

    def update_poi_listbox(self):
        """Updates the POI listbox with the current POIs."""
        self.poi_listbox.delete(0, tk.END)
        for name in self.pois.keys():
            self.poi_listbox.insert(tk.END, name)

    def add_poi_gui(self):
        """Opens a window to add a new POI."""
        add_window = tk.Toplevel(self.root)
        add_window.title("Add POI")

        tk.Label(add_window, text="Name:").grid(row=0, column=0)
        tk.Label(add_window, text="X:").grid(row=1, column=0)
        tk.Label(add_window, text="Y:").grid(row=2, column=0)
        tk.Label(add_window, text="Radius:").grid(row=3, column=0)

        name_entry = tk.Entry(add_window)
        x_entry = tk.Entry(add_window)
        y_entry = tk.Entry(add_window)
        radius_entry = tk.Entry(add_window)

        name_entry.grid(row=0, column=1)
        x_entry.grid(row=1, column=1)
        y_entry.grid(row=2, column=1)
        radius_entry.grid(row=3, column=1)

        def add_poi_action():
            name = name_entry.get()
            try:
                x = float(x_entry.get())
                y = float(y_entry.get())
                radius = float(radius_entry.get())
            except ValueError:
                messagebox.showerror("Input Error", "Please enter valid numerical values for X, Y, and Radius.")
                return

            if name in self.pois:
                messagebox.showerror("Duplicate POI", f"A POI named '{name}' already exists.")
                return

            self.pois[name] = POI(name, x, y, radius)
            self.save_pois(self.poi_file)
            self.update_poi_listbox()
            self.get_logger().info(f"Added POI '{name}' at ({x}, {y}) with radius {radius}")
            add_window.destroy()

        tk.Button(add_window, text="Add", command=add_poi_action).grid(row=4, column=0, columnspan=2)

    def delete_poi_gui(self):
        """Deletes the selected POI."""
        selection = self.poi_listbox.curselection()
        if not selection:
            messagebox.showwarning("No Selection", "Please select a POI to delete.")
            return

        poi_name = self.poi_listbox.get(selection[0])
        if messagebox.askyesno("Confirm Delete", f"Are you sure you want to delete POI '{poi_name}'?"):
            del self.pois[poi_name]
            self.save_pois(self.poi_file)
            self.update_poi_listbox()
            self.get_logger().info(f"Deleted POI '{poi_name}'")

    def navigate_to_poi_gui(self):
        """Navigates to the selected POI."""
        selection = self.poi_listbox.curselection()
        if not selection:
            messagebox.showwarning("No Selection", "Please select a POI to navigate to.")
            return

        poi_name = self.poi_listbox.get(selection[0])
        self.navigate_to_poi(poi_name)

    def navigate_to_poi(self, poi_name):
        """Navigates to the specified POI in a separate thread."""
        if poi_name in self.pois:
            threading.Thread(target=self._navigate_to_poi_thread, args=(poi_name,), daemon=True).start()
        else:
            self.get_logger().warn(f"POI '{poi_name}' not found.")

    def _navigate_to_poi_thread(self, poi_name):
        """Thread target for navigating to a POI."""
        poi = self.pois[poi_name]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position = poi.position
        goal_pose.pose.orientation = self.yaw_to_quaternion(0.0)

        self.navigator.goToPose(goal_pose)
        self.get_logger().info(f"Navigating to POI: {poi_name} at ({poi.position.x}, {poi.position.y})")

        # Wait for the navigation to complete
        result = self.navigator.waitUntilNav2TaskComplete()
        if result == True:
            messagebox.showinfo("Navigation Complete", f"Reached POI '{poi_name}'.")
        else:
            messagebox.showwarning("Navigation Failed", f"Failed to reach POI '{poi_name}'.")

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

    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(poi_manager,), daemon=True)
    ros_thread.start()

    # Start the Tkinter GUI in the main thread
    poi_manager.start_gui()

    # When the GUI is closed, shut down ROS2
    poi_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
