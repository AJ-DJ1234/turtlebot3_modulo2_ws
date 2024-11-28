#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox, Canvas
from PIL import Image, ImageTk
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image as RosImage
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading
import subprocess
import os
import sys

class HazmatInterface(Node):
    def __init__(self):
        super().__init__('interface_node')
        self.bridge = CvBridge()

        # Suscriptor para imágenes de la cámara
        self.image_subscriber = self.create_subscription(
            RosImage,
            '/hazmat_img',
            self.image_callback,
            10
        )

        # Suscriptor para los marcadores de hazmat
        self.marker_subscriber = self.create_subscription(
            MarkerArray,
            '/hazmat_markers',
            self.marker_callback,
            10
        )

        # Cliente para guardar el mapa
        self.client = self.create_client(Trigger, 'save_map')
        self.timer = self.create_timer(10, self.call_save_map_service)

        # Inicializa las variables de estado
        self.markers = []
        self.camera_tk_image = None

        # Variable para gestionar el proceso de exploración
        self.exploration_process = None

    def call_save_map_service(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Service not available')
            return

        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.save_map_response_callback)

    def save_map_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(response.message)
                self.update_map_view()
            else:
                self.get_logger().warning(response.message)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.update_camera_view(cv_image)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def update_camera_view(self, cv_image):
        pil_image = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        self.camera_tk_image = ImageTk.PhotoImage(pil_image)
        camera_label.config(image=self.camera_tk_image)

    def update_map_view(self):
        try:
            img_path = os.path.expanduser('~/Workspaces/turtlebot3_ws/map_slam/map.pgm')
            img = Image.open(img_path)
            resized_map = img.resize((500, 500), Image.ANTIALIAS)
            map_image = ImageTk.PhotoImage(resized_map)
            map_canvas.create_image(0, 0, anchor="nw", image=map_image)
            map_canvas.image = map_image
        except FileNotFoundError:
            self.get_logger().error('Failed to load saved map')

    def marker_callback(self, msg):
        self.markers = msg.markers
        self.update_map_with_markers()
        markers_list.delete(0, tk.END)  # Limpia la lista de marcadores
        for marker in msg.markers:
            markers_list.insert(tk.END, f"Hazmat {marker.id}")

    def update_map_with_markers(self):
        # No eliminar los marcadores anteriores
        for marker in self.markers:
            # Ajusta la escala y posición según tu mapa
            x_pixel = 250 + marker.pose.position.x * 100  # Ajustar según escala
            y_pixel = 250 - marker.pose.position.y * 100
            map_canvas.create_oval(
                x_pixel - 5, y_pixel - 5, x_pixel + 5, y_pixel + 5,
                fill="red", outline="black", width=1, tags="marker"
            )

def load_map():
    try:
        img_path = os.path.expanduser('~/Workspaces/turtlebot3_ws/map_slam/map.pgm')
        img = Image.open(img_path)
        resized_map = img.resize((500, 500), Image.ANTIALIAS)
        map_image = ImageTk.PhotoImage(resized_map)
        map_canvas.create_image(0, 0, anchor="nw", image=map_image)
        map_canvas.image = map_image
    except FileNotFoundError:
        messagebox.showerror("Error", "Mapa no encontrado. Asegúrate de que el archivo map.pgm está en el directorio correcto.")

def start_exploration():
    global node
    if node.exploration_process is None or node.exploration_process.poll() is not None:
        try:
            # Ajusta el comando según cómo se inicie tu nodo de exploración
            # Por ejemplo, si es un nodo ROS2, podrías usar `ros2 run`
            exploration_command = ['ros2', 'launch', 'explore_lite', 'explore.launch.py']
            node.exploration_process = subprocess.Popen(exploration_command)
            update_status("Explorando")
            messagebox.showinfo("Exploración", "Exploración iniciada.")
        except FileNotFoundError:
            messagebox.showerror("Error", "No se encontró el ejecutable para la exploración.")
        except Exception as e:
            messagebox.showerror("Error", f"Error al iniciar la exploración: {e}")
    else:
        messagebox.showwarning("Advertencia", "La exploración ya está en curso.")

def stop_exploration():
    global node
    if node.exploration_process and node.exploration_process.poll() is None:
        node.exploration_process.terminate()
        update_status("Exploración detenida")
        messagebox.showinfo("Exploración", "Exploración detenida.")
    else:
        messagebox.showwarning("Advertencia", "No hay exploración en curso.")

def navigate_to_hazmat(marker):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose = marker.pose
    # Aquí puedes implementar el cliente de acción para enviar el objetivo
    # Por ejemplo, usando rclpy.action para NavigateToPose
    update_status("Navegando")
    messagebox.showinfo("Navegación", f"Navegando al Hazmat {marker.id}")

def select_hazmat(event):
    selected_index = markers_list.curselection()
    if selected_index:
        marker = node.markers[selected_index[0]]
        navigate_to_hazmat(marker)
    else:
        messagebox.showwarning("Advertencia", "No se seleccionó ningún Hazmat.")

def add_hazmat_marker():
    # Esta función ya no es necesaria ya que los Hazmats se agregan automáticamente
    pass

def update_status(new_status):
    status_label.config(text=f"Estado: {new_status}")

def on_closing():
    if messagebox.askokcancel("Salir", "¿Seguro que deseas salir?"):
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error al cerrar ROS2: {e}")
        finally:
            # Terminar el proceso de exploración si está activo
            if node.exploration_process and node.exploration_process.poll() is None:
                node.exploration_process.terminate()
            root.destroy()

def create_interface():
    global root, map_canvas, camera_label, markers_list, status_label

    root = tk.Tk()
    root.title("Interfaz Hazmat")
    root.protocol("WM_DELETE_WINDOW", on_closing)

    # Configurar el grid para que sea responsivo
    root.rowconfigure(0, weight=1)
    root.columnconfigure(0, weight=1)
    root.columnconfigure(1, weight=1)

    # Frames principales
    map_frame = ttk.Frame(root, padding="10")
    map_frame.grid(row=0, column=0, sticky="nsew")
    camera_frame = ttk.Frame(root, padding="10")
    camera_frame.grid(row=0, column=1, sticky="nsew")
    controls_frame = ttk.Frame(root, padding="10")
    controls_frame.grid(row=1, column=0, columnspan=2, sticky="ew")

    # Canvas para el mapa
    global map_canvas
    map_canvas = Canvas(map_frame, width=500, height=500, bg="white")
    map_canvas.pack()

    # Botón para cargar el mapa
    load_map_button = ttk.Button(map_frame, text="Cargar Mapa", command=load_map)
    load_map_button.pack(pady=5)

    # Etiqueta para la cámara
    camera_label = ttk.Label(camera_frame)
    camera_label.pack()

    # Lista de Hazmat
    markers_list = tk.Listbox(controls_frame, height=10)
    markers_list.pack(side="left", padx=10, pady=10)
    markers_list.bind("<Double-1>", select_hazmat)

    # Botones de control
    start_button = ttk.Button(controls_frame, text="Iniciar Exploración", command=start_exploration)
    start_button.pack(side="left", padx=10)

    stop_button = ttk.Button(controls_frame, text="Detener Exploración", command=stop_exploration)
    stop_button.pack(side="left", padx=10)

    # Etiqueta de estado
    status_label = ttk.Label(controls_frame, text="Estado: Esperando acción")
    status_label.pack(side="left", padx=10)

    # Ejecutar la interfaz de Tkinter
    root.mainloop()

def main():
    rclpy.init()
    global node
    node = HazmatInterface()

    # Crear la interfaz
    interface_thread = threading.Thread(target=create_interface)
    interface_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
