# !/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class HazmatInterface(Node):
    def __init__(self):
        super().__init__('interface_node')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(RosImage,
                                                        '/hazmat_img',
                                                        self.image_callback,
                                                        10)
        self.mark

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

def load_map():
    try:
        img = Image.open('/home/ajdj/Workspaces/turtlebot3_ws/map_slam/map.pgm')
        tk_img = ImageTk.PhotoImage(img)
        map_label.config(image=tk_img)
        map_label.image = tk_img
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
    root = tk.Tk()
    root.title("Interfaz Hazmat")

    # Crear los frames
    map_frame = ttk.Frame(root, padding="10")
    map_frame.grid(row=0, column=0, sticky="nsew")
    camera_frame = ttk.Frame(root, padding="10")
    camera_frame.grid(row=0, column=1, sticky="nsew")
    controls_frame = ttk.Frame(root, padding="10")
    controls_frame.grid(row=1, column=0, columnspan=2, sticky="ew")

    # Etiqueta para el mapa
    global map_label
    map_label = ttk.Label(map_frame)
    map_label.pack()

    # Botón para cargar el mapa
    load_map_button = ttk.Button(map_frame, text="Cargar Mapa", command=load_map)
    load_map_button.pack()

    # Etiqueta para la cámara
    global camera_label
    camera_label = ttk.Label(camera_frame)
    camera_label.pack()

    # Campo de entrada y botón para agregar marcadores de hazmat
    global hazmat_id_entry
    hazmat_id_entry = ttk.Entry(controls_frame)
    hazmat_id_entry.pack(side="left", padx=5)
    add_marker_button = ttk.Button(controls_frame, text="Agregar Hazmat", command=add_hazmat_marker)
    add_marker_button.pack(side="left", padx=5)

    # Lista de marcadores
    global markers_list
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
