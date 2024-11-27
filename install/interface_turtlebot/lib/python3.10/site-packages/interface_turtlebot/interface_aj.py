#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox, Canvas
from PIL import Image, ImageTk
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image as RosImage
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading


class HazmatInterface(Node):
    def __init__(self):
        super().__init__('interface_node')
        self.bridge = CvBridge()

        # Suscriptor para imágenes de la cámara
        self.image_subscriber = self.create_subscription(RosImage,
                                                        '/hazmat_img',
                                                        self.image_callback,
                                                        10)

        # Suscriptor para los marcadores de hazmat
        self.marker_subscriber = self.create_subscription(MarkerArray,
                                                        '/hazmat_markers',
                                                        self.marker_callback,
                                                        10)

        # Cliente para guardar el mapa
        self.client = self.create_client(Trigger, 'save_map')
        self.timer = self.create_timer(10, self.call_save_map_service)

        # Inicializa las variables de estado
        self.markers = []
        self.camera_tk_image = None

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
            img = Image.open('/home/ajdj/Workspaces/turtlebot3_ws/map_slam/map.pgm')
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
        for marker in self.markers:
            x_pixel = 250 + marker.pose.position.x * 100  # Ajustar según escala
            y_pixel = 250 - marker.pose.position.y * 100
            map_canvas.create_oval(x_pixel - 5, y_pixel - 5, x_pixel + 5, y_pixel + 5,
                                   fill="red", outline="black", width=1)

def load_map():
    try:
        img = Image.open('/home/ajdj/Workspaces/turtlebot3_ws/map_slam/map.pgm')
        resized_map = img.resize((500, 500), Image.ANTIALIAS)
        map_image = ImageTk.PhotoImage(resized_map)
        map_canvas.create_image(0, 0, anchor="nw", image=map_image)
        map_canvas.image = map_image
    except FileNotFoundError:
        messagebox.showerror("Error", "Mapa no encontrado. Asegúrate de que el archivo map.pgm está en el directorio correcto.")

def start_exploration():
    update_status("Explorando")
    messagebox.showinfo("Exploración", "Exploración iniciada.")
    # Aquí puedes agregar el código para iniciar la exploración

def navigate_to_hazmat(marker):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose = marker.pose
    # Aquí puedes implementar el cliente de acción para enviar el objetivo
    messagebox.showinfo("Navegación", f"Navegando al Hazmat {marker.id}")

def select_hazmat(event):
    selected_index = markers_list.curselection()
    if selected_index:
        marker = node.markers[selected_index[0]]
        navigate_to_hazmat(marker)
    else:
        messagebox.showwarning("Advertencia", "No se seleccionó ningún Hazmat.")

def add_hazmat_marker():
    hazmat_id = hazmat_id_entry.get()
    if hazmat_id:
        markers_list.insert(tk.END, hazmat_id)
        hazmat_id_entry.delete(0, tk.END)
    else:
        messagebox.showwarning("Advertencia", "ID del hazmat no puede estar vacío.")

def update_status(new_status):
    status_label.config(text=f"Estado: {new_status}")

def on_closing():
    if messagebox.askokcancel("Salir", "¿Seguro que deseas salir?"):
        try:
            if rclpy.ok():  # Verifica si el contexto de ROS2 está activo
                rclpy.shutdown()
        except Exception as e:
            print(f"Error al cerrar ROS2: {e}")
        finally:
            root.destroy()

def create_interface():
    global root, map_canvas, camera_label, hazmat_id_entry, markers_list, status_label

    root = tk.Tk()
    root.title("Interfaz Hazmat")
    root.protocol("WM_DELETE_WINDOW", on_closing)

    # Frames principales
    map_frame = ttk.Frame(root, padding="10")
    map_frame.grid(row=0, column=0, sticky="nsew")
    camera_frame = ttk.Frame(root, padding="10")
    camera_frame.grid(row=0, column=1, sticky="nsew")
    controls_frame = ttk.Frame(root, padding="10")
    controls_frame.grid(row=1, column=0, columnspan=2, sticky="ew")

    # Canvas para el mapa
    map_canvas = Canvas(map_frame, width=500, height=500)
    map_canvas.pack()

    # Etiqueta para la cámara
    camera_label = ttk.Label(camera_frame)
    camera_label.pack()

    # Campo de entrada y botón para agregar Hazmat
    hazmat_id_entry = ttk.Entry(controls_frame)
    hazmat_id_entry.pack(side="left", padx=5)
    add_marker_button = ttk.Button(controls_frame, text="Agregar Hazmat", command=add_hazmat_marker)
    add_marker_button.pack(side="left", padx=5)

    # Lista de Hazmat
    markers_list = tk.Listbox(controls_frame, height=5)
    markers_list.pack(side="left", fill="y", padx=5)
    markers_list.bind('<<ListboxSelect>>', select_hazmat)

    # Botón para iniciar exploración
    start_button = ttk.Button(controls_frame, text="Iniciar Exploración", command=start_exploration)
    start_button.pack(side="right")

    # Estado del robot
    status_label = ttk.Label(root, text="Estado: Inactivo", anchor="w")
    status_label.grid(row=2, column=0, columnspan=2, sticky="ew")

    root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    global node
    node = HazmatInterface()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    create_interface()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
