#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import socket
import sys
import zlib

sys.path.append('/home/ajdj/git/DeepHAZMAT')
from deep_hazmat import DeepHAZMAT

class CamaraServer:
    def __init__(self):
        # Inicializar el modelo DeepHAZMAT
        net_directory = '/home/ajdj/git/DeepHAZMAT/net'
        self.model = DeepHAZMAT(
            k=0,
            min_confidence=0.95,
            nms_threshold=0.2,
            segmentation_enabled=True,
            net_directory=net_directory
        )

        # Configurar el servidor socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('192.168.50.155', 10005))
        self.server_socket.listen()
        print('Servidor escuchando en el puerto 10005')

    def process_request(self):
        try:
            conn, addr = self.server_socket.accept()
            with conn:
                print(f'Conectado por {addr}')
                
                # Recibir el tamaño de la imagen comprimida
                img_size_data = conn.recv(4)
                if not img_size_data:
                    print("Error: No se recibió el tamaño de la imagen.")
                    return

                img_size = int.from_bytes(img_size_data, byteorder='little')
                print(f'Tamaño de la imagen recibido: {img_size} bytes')

                b_imagen = bytearray()
                while len(b_imagen) < img_size:
                    data = conn.recv(4096)
                    if not data:
                        print("Error: Se interrumpió la recepción de datos.")
                        return
                    b_imagen += data

                if len(b_imagen) != img_size:
                    print(f"Error: Tamaño de la imagen recibido ({len(b_imagen)}) no coincide con el tamaño esperado ({img_size}).")
                    return

                # Descomprimir y decodificar la imagen
                img_enc = zlib.decompress(b_imagen)
                img_dec = np.ndarray(shape=(len(img_enc),), dtype='uint8', buffer=img_enc)
                color = cv.imdecode(img_dec, cv.IMREAD_COLOR)

                if color is None:
                    print("Error: No se pudo decodificar la imagen.")
                    return

                # Procesar detecciones de HAZMAT
                hazmat_names = []
                for hazmat in self.model.update(color):
                    hazmat.draw(color, padding=0.1)
                    hazmat_names.append(hazmat.name)

                # Formatear el mensaje de nombres de HAZMAT
                if hazmat_names:
                    names_str = ", ".join(hazmat_names)
                else:
                    names_str = "No detections"

                # Enviar tamaño de los nombres de HAZMAT
                name_size = len(names_str.encode('utf-8'))
                conn.sendall(name_size.to_bytes(4, byteorder='little'))

                # Enviar nombres de HAZMAT
                conn.sendall(names_str.encode('utf-8'))
                print(f"Nombres de HAZMAT enviados: {names_str}")

                # Codificar y enviar la imagen procesada de vuelta al cliente
                _, img_encoded = cv.imencode('.jpg', color)
                conn.sendall(len(img_encoded).to_bytes(4, byteorder='little'))
                conn.sendall(img_encoded.tobytes())

        except Exception as e:
            print(f'Error en el procesamiento de la solicitud: {e}')

def main():
    server = CamaraServer()
    try:
        while True:
            server.process_request()
    except KeyboardInterrupt:
        print("Servidor detenido.")

if __name__ == '__main__':
    main()

