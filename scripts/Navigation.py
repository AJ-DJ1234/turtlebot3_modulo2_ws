#!/usr/bin/env python3
"""
DescripciÃ³n General:

Este nodo de ROS2, denominado 'explore_node', estÃ¡ diseÃ±ado para permitir que un robot realice una exploraciÃ³n autÃ³noma de un entorno desconocido utilizando el paquete Nav2. El nodo suscribe al mapa de ocupaciÃ³n publicado en '/map', identifica las celdas frontier (es decir, celdas libres adyacentes a espacios desconocidos), selecciona un objetivo de navegaciÃ³n basado en criterios de distancia y envÃ­a este objetivo al sistema de navegaciÃ³n. El proceso se repite hasta que no se encuentren mÃ¡s frontiers, indicando que la exploraciÃ³n ha finalizado o que se requiere una actualizaciÃ³n del mapa.

**Instrucciones de EjecuciÃ³n**:

Para ejecutar correctamente este nodo, es necesario abrir tres terminales diferentes y ejecutar los siguientes comandos en cada una de ellas:

1. **Terminal 1**: Iniciar el sistema de mapeo SLAM Toolbox.

    ```bash
    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false map_update_interval:=0.5
    ```

    **DescripciÃ³n**: Este comando lanza el paquete SLAM Toolbox en modo asÃ­ncrono para realizar el mapeo en lÃ­nea. El parÃ¡metro `use_sim_time:=false` indica que se usarÃ¡ el tiempo real en lugar de simulado, y `map_update_interval:=0.5` establece el intervalo de actualizaciÃ³n del mapa a 0.5 segundos.

2. **Terminal 2**: Iniciar el stack de navegaciÃ³n Navigation2 para el TurtleBot3 Burger.

    ```bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=false params_file:=/opt/ros/humble/share/turtlebot3_navigation2/param/burger.yaml use_amcl:=false
    ```

    **DescripciÃ³n**: Este comando lanza el stack de navegaciÃ³n Navigation2 con parÃ¡metros especÃ­ficos para el robot TurtleBot3 Burger. El parÃ¡metro `use_sim_time:=false` usa tiempo real, `params_file` especifica el archivo de configuraciÃ³n de parÃ¡metros, y `use_amcl:=false` desactiva el uso de AMCL (Adaptive Monte Carlo Localization).

3. **Terminal 3**: Ejecutar el nodo de exploraciÃ³n personalizado.

    ```bash
    ros2 run turtlebot3_explore explore
    ```

    **DescripciÃ³n**: Este comando ejecuta el nodo de exploraciÃ³n (`explore`) que estÃ¡ diseÃ±ado para que el TurtleBot3 explore el entorno autÃ³nomamente siguiendo la lÃ³gica implementada en el cÃ³digo.

Funcionamiento:

1. **InicializaciÃ³n**: El nodo espera a que el sistema Nav2 estÃ© activo y listo para recibir objetivos de navegaciÃ³n. Configura un suscriptor para el mapa de ocupaciÃ³n y prepara el sistema de transformaciones TF para obtener la pose actual del robot.

2. **Bucle de ExploraciÃ³n**:
   - **ObtenciÃ³n de la Pose Actual**: Utiliza las transformaciones TF para determinar la posiciÃ³n y orientaciÃ³n actuales del robot en el marco de referencia del mapa.
   
   - **IdentificaciÃ³n de Frontiers**: Procesa el mapa de ocupaciÃ³n para identificar celdas libres que estÃ¡n adyacentes a espacios desconocidos, conocidas como frontiers.
   
   - **SelecciÃ³n de Objetivo**: Entre las frontiers identificadas, selecciona la mÃ¡s lejana que cumpla con una distancia mÃ­nima desde la posiciÃ³n actual del robot para evitar objetivos demasiado cercanos.
   
   - **NavegaciÃ³n hacia el Objetivo**: EnvÃ­a el objetivo seleccionado al sistema de navegaciÃ³n Nav2 y monitorea el progreso de la navegaciÃ³n. Si el robot alcanza el objetivo o si se excede un tiempo de espera, la tarea de navegaciÃ³n se cancela y se procede a seleccionar un nuevo objetivo.
   
   - **RepeticiÃ³n**: El proceso se repite hasta que no se encuentren mÃ¡s frontiers, momento en el cual se detiene la exploraciÃ³n o se espera una actualizaciÃ³n del mapa.

3. **Manejo de Errores**: El nodo maneja excepciones relacionadas con la obtenciÃ³n de transformaciones TF y la navegaciÃ³n, asegurando que el robot no se detenga inesperadamente ante problemas en la comunicaciÃ³n o en los datos del mapa.

Este nodo es Ãºtil para aplicaciones de robÃ³tica mÃ³vil donde se requiere una exploraciÃ³n sistemÃ¡tica de un entorno desconocido, como en la cartografÃ­a, bÃºsqueda y rescate, o tareas de vigilancia.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import math
import numpy as np

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class Explore(Node):
    def __init__(self):
        super().__init__('explore_node')
        self.navigator = BasicNavigator()
        self.get_logger().info("Esperando a que Nav2 se active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Â¡Nav2 estÃ¡ listo para su uso!")

        self.minimum_goal_distance = 0.5  # Distancia mÃ­nima desde la posiciÃ³n actual
        self.goal_reached_threshold = 0.2  # Umbral para considerar que se alcanzÃ³ el objetivo

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_map = None
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Iniciar el bucle de exploraciÃ³n intentando el primer objetivo
        self.get_logger().info("Iniciando exploraciÃ³n...")
        self.attempt_new_goal()

    def get_current_pose(self):
        try:
            now = rclpy.time.Time()
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',  # o 'base_link' dependiendo del marco del robot
                time=now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            current_pose = PoseStamped()
            current_pose.header = transform.header
            current_pose.pose.position.x = transform.transform.translation.x
            current_pose.pose.position.y = transform.transform.translation.y
            current_pose.pose.position.z = transform.transform.translation.z
            current_pose.pose.orientation = transform.transform.rotation
            self.get_logger().debug(f"Pose actual: x={current_pose.pose.position.x:.2f}, y={current_pose.pose.position.y:.2f}")
            return current_pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"No se pudo obtener la pose actual: {e}")
            return None

    def map_callback(self, data):
        self.latest_map = data  # Almacenar el mapa mÃ¡s reciente para usar en la selecciÃ³n de objetivos
        self.get_logger().info("Datos del mapa recibidos.")

    def attempt_new_goal(self):
        self.get_logger().info("Intentando encontrar un nuevo objetivo...")
        if self.latest_map is None:
            self.get_logger().info("Mapa no recibido aÃºn. Esperando datos del mapa...")
            # Programar para intentar nuevamente despuÃ©s de un breve retraso
            self.create_timer(1.0, self.attempt_new_goal)
            return

        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().warn("La pose actual no estÃ¡ disponible aÃºn. Esperando pose...")
            # Programar para intentar nuevamente despuÃ©s de un breve retraso
            self.create_timer(1.0, self.attempt_new_goal)
            return

        # Encontrar frontiers (celdas adyacentes a espacio desconocido)
        frontier_indices = self.find_frontiers(self.latest_map)
        self.get_logger().info(f"NÃºmero de frontiers encontradas: {len(frontier_indices)}")

        if not frontier_indices:
            self.get_logger().warn("No se encontraron frontiers. ExploraciÃ³n completa o se necesita una actualizaciÃ³n del mapa.")
            # Esperar antes de intentar nuevamente
            self.create_timer(2.0, self.attempt_new_goal)
            return

        # Seleccionar un punto frontier
        selected_index = self.select_frontier(frontier_indices, current_pose)
        if selected_index is None:
            self.get_logger().warn("No se pudo seleccionar una frontier vÃ¡lida.")
            # Esperar antes de intentar nuevamente
            self.create_timer(2.0, self.attempt_new_goal)
            return

        # Calcular la posiciÃ³n del objetivo
        width = self.latest_map.info.width
        resolution = self.latest_map.info.resolution
        origin_x = self.latest_map.info.origin.position.x
        origin_y = self.latest_map.info.origin.position.y

        row = selected_index // width
        col = selected_index % width

        self.x = (col * resolution) + origin_x + resolution / 2.0
        self.y = (row * resolution) + origin_y + resolution / 2.0

        self.get_logger().info(f"Nuevo objetivo en x: {self.x:.2f}, y: {self.y:.2f}")
        self.set_goal()

    def set_goal(self):
        """Establecer un objetivo de navegaciÃ³n para el robot."""
        self.get_logger().info(f"Estableciendo objetivo en x: {self.x:.2f}, y: {self.y:.2f}")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = self.x
        goal_pose.pose.position.y = self.y
        goal_pose.pose.orientation.w = 1.0  # OrientaciÃ³n hacia adelante

        # Enviar el objetivo al navigator
        self.navigator.goToPose(goal_pose)
        self.get_logger().info("Objetivo enviado al navigator.")

        # Monitorear la tarea de navegaciÃ³n
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                distance = feedback.distance_remaining
                self.get_logger().info(f"Distancia restante: {distance:.2f} metros")

                # Verificar si estÃ¡ dentro del umbral del objetivo
                if distance <= self.goal_reached_threshold:
                    self.navigator.cancelTask()
                    self.get_logger().info("Umbral del objetivo alcanzado. Cancelando navegaciÃ³n.")
                    break

                # Opcional: AÃ±adir un tiempo de espera o condiciÃ³n de cancelaciÃ³n
                if feedback.navigation_time.sec > 300:  # por ejemplo, tiempo de espera despuÃ©s de 5 minutos
                    self.navigator.cancelTask()
                    self.get_logger().info("Tarea de navegaciÃ³n cancelada por tiempo de espera.")
                    break

            rclpy.spin_once(self, timeout_sec=0.1)

        # Verificar el resultado de la tarea de navegaciÃ³n
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED or distance <= self.goal_reached_threshold:
            self.get_logger().info("Â¡Objetivo alcanzado con Ã©xito!")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("El objetivo fue cancelado.")
        elif result == TaskResult.FAILED:
            self.get_logger().warn("El objetivo fallÃ³.")
        else:
            self.get_logger().warn("El objetivo tiene un resultado desconocido.")

        # Dar tiempo para que el robot se asiente
        time.sleep(1.0)

        # DespuÃ©s de completar el objetivo, intentar encontrar uno nuevo
        self.attempt_new_goal()

    def find_frontiers(self, map_data):
        """Encontrar celdas frontier en el mapa."""
        width = map_data.info.width
        height = map_data.info.height
        data = np.array(map_data.data).reshape((height, width))
        data = np.flipud(data)  # Voltear los datos verticalmente para alinear con el sistema de coordenadas de ROS
        self.get_logger().info(f"TamaÃ±o del mapa: ancho={width}, alto={height}")
        frontiers = []

        for y in range(height):
            for x in range(width):
                if data[y][x] == 0:  # Espacio libre
                    # Verificar vecinos 4-conectados
                    neighbors = self.get_neighbors(x, y, width, height)
                    for nx, ny in neighbors:
                        if data[ny][nx] == -1:  # Espacio desconocido
                            frontiers.append(y * width + x)
                            break  # No es necesario verificar otros vecinos
        self.get_logger().info(f"Total de frontiers identificadas: {len(frontiers)}")
        return frontiers

    def get_neighbors(self, x, y, width, height):
        """Obtener celdas vecinas vÃ¡lidas (4-conectividad)."""
        neighbors = []
        if x > 0:
            neighbors.append((x - 1, y))
        if x < width - 1:
            neighbors.append((x + 1, y))
        if y > 0:
            neighbors.append((x, y - 1))
        if y < height - 1:
            neighbors.append((x, y + 1))
        return neighbors

    def select_frontier(self, frontier_indices, current_pose):
        """Seleccionar una celda frontier a la cual navegar."""
        if not frontier_indices:
            return None

        width = self.latest_map.info.width
        resolution = self.latest_map.info.resolution
        origin_x = self.latest_map.info.origin.position.x
        origin_y = self.latest_map.info.origin.position.y

        # Calcular distancias a cada frontier
        distances = []
        for index in frontier_indices:
            row = index // width
            col = index % width
            x = (col * resolution) + origin_x + resolution / 2.0
            y = (row * resolution) + origin_y + resolution / 2.0
            distance = math.hypot(
                x - current_pose.pose.position.x,
                y - current_pose.pose.position.y
            )
            distances.append((distance, index))

        # Registrar la cantidad de distancias calculadas
        self.get_logger().info(f"Distancias calculadas a {len(distances)} frontiers.")

        # Ordenar frontiers por distancia (las mÃ¡s lejanas primero)
        distances.sort(reverse=True)

        # Seleccionar la frontier mÃ¡s lejana que estÃ© al menos a la distancia mÃ­nima
        for distance, index in distances:
            if distance >= self.minimum_goal_distance:
                self.get_logger().info(f"Selected frontier at index {index} con distancia {distance:.2f}")
                return index  # Retornar el Ã­ndice de la frontier seleccionada

        self.get_logger().warn("No se encontrÃ³ una frontier que supere la distancia mÃ­nima del objetivo.")
        return None  # No se encontrÃ³ una frontier vÃ¡lida

def main(args=None):
    rclpy.init(args=args)
    explore_node = Explore()

    try:
        rclpy.spin(explore_node)
    except KeyboardInterrupt:
        pass
    finally:
        explore_node.navigator.lifecycleShutdown()
        explore_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()