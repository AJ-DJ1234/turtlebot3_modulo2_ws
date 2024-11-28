import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import threading


class HazmatInterface(Node):
    def __init__(self):
        super().__init__('hazmat_interface')
        self.bridge = CvBridge()
        self.camera_tk_image = None

        # Crear el subscritor al tópico de imágenes
        self.subscription = self.create_subscription(
            ROSImage,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Inicializar la interfaz gráfica
        self.root = tk.Tk()
        self.root.title("Hazmat Interface")
        self.root.geometry("800x600")

        # Crear etiquetas y botones
        self.camera_label = tk.Label(self.root)
        self.camera_label.pack(pady=10)
        self.quit_button = ttk.Button(self.root, text="Salir", command=self.quit_interface)
        self.quit_button.pack(pady=10)

        # Iniciar el bucle de eventos de Tkinter en un hilo separado
        self.tk_thread = threading.Thread(target=self.start_tk, daemon=True)
        self.tk_thread.start()

    def start_tk(self):
        try:
            self.root.mainloop()
        except Exception as e:
            self.get_logger().error(f"Error en el bucle de Tkinter: {e}")

    def quit_interface(self):
        self.root.quit()

    def image_callback(self, msg):
        """Callback para procesar las imágenes recibidas."""
        try:
            # Convertir la imagen de ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.update_camera_view(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error al procesar la imagen: {e}")

    def update_camera_view(self, cv_image):
        """Actualiza la imagen mostrada en la interfaz."""
        try:
            # Convertir la imagen de OpenCV a PIL
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(rgb_image)

            # Convertir la imagen de PIL a ImageTk.PhotoImage
            self.camera_tk_image = ImageTk.PhotoImage(pil_image)

            # Actualizar la etiqueta de la cámara
            self.camera_label.config(image=self.camera_tk_image)
        except Exception as e:
            self.get_logger().error(f"Error al actualizar la vista de la cámara: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HazmatInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interfaz finalizada por el usuario.")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
