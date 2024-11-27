#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2 as cv
import numpy as np
import socket
import zlib
from cv_bridge import CvBridge

class CamaraClient(Node):
    def __init__(self):
        super().__init__('camara_client')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        
        self.hazmat_publisher = self.create_publisher(String, 'hazmat_publisher', 10)
        self.image_publisher = self.create_publisher(Image, '/hazmat_img', 10)
        
        self.server_address = ('127.0.0.1', 10005)

    def image_callback(self, msg):
        try:
            imagen = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error al convertir la imagen: {e}")
            return

        if imagen is None or imagen.size == 0:
            self.get_logger().error("Error: La imagen está vacía después de la conversión.")
            return
        
        # Reducir la resolución de la imagen
        imagen = cv.resize(imagen, (320, 240))  # Ajustar para menor tiempo de procesamiento

        # Codificar y comprimir la imagen
        result, img_enc = cv.imencode('.jpg', imagen)
        if not result:
            self.get_logger().error("Error al codificar la imagen.")
            return

        img_compressed = zlib.compress(img_enc)

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect(self.server_address)

                # Enviar el tamaño de la imagen comprimida y la imagen
                s.sendall(len(img_compressed).to_bytes(4, byteorder='little'))
                s.sendall(img_compressed)

                # **Recibir y procesar los nombres de HAZMAT**
                name_size_data = s.recv(4)
                if not name_size_data:
                    self.get_logger().error("Error: No se recibió el tamaño de los nombres de HAZMAT.")
                    return

                name_size = int.from_bytes(name_size_data, byteorder='little')
                hazmat_names_data = s.recv(name_size).decode('utf-8')

                # Publicar el mensaje de detección
                msg = String()
                msg.data = hazmat_names_data
                self.hazmat_publisher.publish(msg)
                
                if hazmat_names_data == "No detections":
                    self.get_logger().info("No se detectaron HAZMAT en esta imagen.")
                else:
                    self.get_logger().info(f'Nombres de HAZMAT detectados: {hazmat_names_data}')

                # Recibir y decodificar la imagen procesada
                img_size_data = s.recv(4)
                img_size = int.from_bytes(img_size_data, byteorder='little')
                b_json = bytearray()
                while len(b_json) < img_size:
                    data = s.recv(4096)
                    if not data:
                        self.get_logger().error("Error: Conexión interrumpida mientras se recibían datos.")
                        return
                    b_json.extend(data)

        except Exception as e:
            self.get_logger().error(f"Error en la conexión con el servidor: {e}")
            return

        # Recibir y decodificar la imagen procesada
        img_results = np.frombuffer(b_json, dtype=np.uint8)
        img_decoded = cv.imdecode(img_results, cv.IMREAD_COLOR)
        if img_decoded is None:
            self.get_logger().error("Error: No se pudo decodificar la imagen.")
            return

        # Convertir de BGR a RGB antes de publicarla
        img_decoded_rgb = cv.cvtColor(img_decoded, cv.COLOR_BGR2RGB)

        # Publicar la imagen procesada en el tópico `/hazmat_img`
        img_msg = self.bridge.cv2_to_imgmsg(img_decoded_rgb, encoding="rgb8")
        self.image_publisher.publish(img_msg)



def main(args=None):
    rclpy.init(args=args)
    camara_client = CamaraClient()

    try:
        rclpy.spin(camara_client)
    except KeyboardInterrupt:
        pass
    finally:
        camara_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

