# !/usr/bin/env python3

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

        # Variables para el mapa y los marcadores
        self.markers = []

        self.client = self.create_client(Trigger, 'save_map')
        self.timer = self.create_timer(10, self.call_save_map_service)

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
        tk_image = ImageTk.PhotoImage(pil_image)
        camera_label.config(image=tk_image)
        camera_label.image = tk_image

    def update_map_view(self):
        try:
            img = Image.open('~/my_saved_maps/map.pgm')
            tk_img = ImageTk.PhotoImage(img)
            map_label.config(image=tk_img)
            map_label.image = tk_img
        except FileNotFoundError:
            self.get_logger().error('Failed to load saved map')

    def marker_callback(self, msg):
        # Actualiza la lista de marcadores
        self.markers = msg.markers
        self.update_map_with_markers()
        for marker in msg.markers:
            self.add_marker_to_list(marker)

    def add_marker_to_list(self, marker):
        # Marcadores en la lista
        marker_id = marker.id
        markers_list.insert(tk.END, f"Hazmat {marker_id}")

    def update_map_with_markers(self):
        # Dibuja el mapa y los marcadores
        load_map()
        for marker in self.markers:
            # Convierte las coordenadas de ROS a píxeles
            x_pixel = map_width / 2 + marker.pose.position.x * map_width
            y_pixel = map_height / 2 - marker.pose.position.y * map_height

            # Dibuja un círculo en el Canvas
            canvas.create_oval(
                x_pixel - 5, y_pixel - 5, x_pixel + 5, y_pixel + 5,
                fill="red", outline="black", width=1
            )

def load_map():
    global map_image, map_width, map_height, canvas
    try:
        # Escalar el mapa a las nuevas dimensiones
        img = Image.open('/home/ajdj/Workspaces/turtlebot3_ws/map_slam/map.pgm')
        resized_map = img.resize((700, 700), Image.ANTIALIAS)  # Ajusta al tamaño del canvas
        map_image = ImageTk.PhotoImage(resized_map)
        
        # Redibuja el mapa en el Canvas
        canvas.create_image(0, 0, anchor="nw", image=map_image)
    except FileNotFoundError:
        messagebox.showerror("Error", "Mapa no encontrado. Asegúrate de que el archivo map.pgm está en el directorio correcto.")

def start_exploration():
    messagebox.showinfo("Exploración", "Exploración iniciada.")
    # Aquí puedes agregar el código para iniciar la exploración 
    # Por ejemplo, iniciar un nodo de navegación

def add_hazmat_marker():
    hazmat_id = hazmat_id_entry.get()
    if hazmat_id:
        markers_list.insert(tk.END, hazmat_id)
        hazmat_id_entry.delete(0, tk.END)
    else:
        messagebox.showwarning("Advertencia", "ID del hazmat no puede estar vacío.")

def select_hazmat(event):
    selected_hazmat = markers_list.get(markers_list.curselection())
    messagebox.showinfo("Seleccionar Hazmat", f"Hazmat seleccionado: {selected_hazmat}")
    # Aquí puedes agregar el código para enviar el objetivo de navegación al robot

def create_interface():
    global root, map_label, camera_label, hazmat_id_entry, markers_list, canvas

    root = tk.Tk()
    root.title("Interfaz Hazmat")

    # Crear los frames
    map_frame = ttk.Frame(root, padding="10")
    map_frame.grid(row=0, column=0, sticky="nsew")
    camera_frame = ttk.Frame(root, padding="10")
    camera_frame.grid(row=0, column=1, sticky="nsew")
    controls_frame = ttk.Frame(root, padding="10")
    controls_frame.grid(row=1, column=0, columnspan=2, sticky="ew")

    # Canvas para el mapa
    canvas = Canvas(map_frame, width=500, height=500)
    canvas.pack()

    # Botón para cargar el mapa
    load_map_button = ttk.Button(map_frame, text="Cargar Mapa", command=load_map)
    load_map_button.pack()

    # Etiqueta para la cámara
    camera_label = ttk.Label(camera_frame)
    camera_label.pack()

    # Campo de entrada y botón para agregar marcadores de hazmat
    hazmat_id_entry = ttk.Entry(controls_frame)
    hazmat_id_entry.pack(side="left", padx=5)
    add_marker_button = ttk.Button(controls_frame, text="Agregar Hazmat", command=add_hazmat_marker)
    add_marker_button.pack(side="left", padx=5)

    # Lista de marcadores
    markers_list = tk.Listbox(controls_frame)
    markers_list.pack(side="left", fill="y", padx=5)
    markers_list.bind('<<ListboxSelect>>', select_hazmat)

    # Botón para iniciar la exploración
    start_button = ttk.Button(controls_frame, text="Iniciar Exploración", command=start_exploration)
    start_button.pack(side="right")

    root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = HazmatInterface()
    create_interface()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
