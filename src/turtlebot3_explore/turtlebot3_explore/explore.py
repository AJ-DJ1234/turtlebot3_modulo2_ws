#!/usr/bin/env python3
"""
Enhanced Exploration Node for TurtleBot3 with Slam Toolbox and Nav2.
This script uses BasicNavigator to send navigation goals to frontiers and monitors their status.
Enhancements include:
- Prioritization of frontiers based on information gain and distance.
- Proper marking of visited frontiers.
- Improved filtering based on selection attempts.
- Increased timer interval to reduce processing load.
- Enhanced logging for better debugging.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from scipy.signal import convolve2d


class Explore(Node):
    def __init__(self):
        super().__init__('explore_node')

        # Declare parameters with default values
        self.declare_parameter('info_radius', 1.0)
        self.declare_parameter('frontier_visit_threshold', 0.6)
        self.declare_parameter('max_attempts', 10)
        self.declare_parameter('timer_interval', 2.0)  # Aumentado a 2.0 segundos
        self.declare_parameter('dbscan_eps_multiplier', 1.0)  # Ajustado a 1.0
        self.declare_parameter('dbscan_min_samples', 1)  # Ajustado a 1

        # Get parameter values
        self.info_radius = self.get_parameter('info_radius').get_parameter_value().double_value
        self.frontier_visit_threshold = self.get_parameter('frontier_visit_threshold').get_parameter_value().double_value
        self.max_attempts = self.get_parameter('max_attempts').get_parameter_value().integer_value
        self.timer_interval = self.get_parameter('timer_interval').get_parameter_value().double_value
        self.dbscan_eps_multiplier = self.get_parameter('dbscan_eps_multiplier').get_parameter_value().double_value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').get_parameter_value().integer_value

        # Initialize map data and subscriber
        self.latest_map = None
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize navigator
        self.navigator = BasicNavigator()
        self.get_logger().info("Esperando a que Nav2 esté activo...")

        # Wait until Nav2 is ready
        self.wait_until_nav2_ready()
        self.get_logger().info("Nav2 está listo para usar!")

        # Visited frontiers history
        self.visited_frontiers = []  # List to store visited frontiers
        self.frontier_selection_attempts = {}  # Dictionary to count selection attempts per frontier

        # Publisher to visualize frontiers
        self.frontier_pub = self.create_publisher(MarkerArray, 'frontiers_individuales', 10)

        # Publisher to stop the robot if needed
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Node state
        self.navigation_in_progress = False
        self.current_goal = None  # Para almacenar el objetivo actual

        # Create a timer for the main exploration cycle with adjusted frequency
        self.timer = self.create_timer(self.timer_interval, self.exploration_cycle)

        self.get_logger().info("Inicio de la exploración...")

    def round_coord(self, coord, precision=2):
        """Redondea las coordenadas a una precisión específica."""
        return (round(coord[0], precision), round(coord[1], precision))

    def get_current_pose(self):
        """Retrieve the robot's current pose in the 'map' frame."""
        try:
            now = rclpy.time.Time()
            if not self.tf_buffer.can_transform('map', 'base_link', now):
                self.get_logger().warn("Transform from 'map' to 'base_link' not available.")
                return None

            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                time=now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            current_pose = PoseStamped()
            current_pose.header = transform.header
            current_pose.pose.position.x = transform.transform.translation.x
            current_pose.pose.position.y = transform.transform.translation.y
            current_pose.pose.position.z = transform.transform.translation.z
            current_pose.pose.orientation = transform.transform.rotation
            return current_pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get current pose: {e}")
            return None

    def wait_until_nav2_ready(self):
        """Wait until Nav2 is active and ready for navigation."""
        # Wait until the map is received
        while rclpy.ok() and self.latest_map is None:
            self.get_logger().info("Esperando datos del mapa...")
            rclpy.spin_once(self, timeout_sec=1.0)

        # Wait until the TFs are available
        while rclpy.ok():
            try:
                now = rclpy.time.Time()
                if self.tf_buffer.can_transform('map', 'base_link', now):
                    break
                else:
                    self.get_logger().info("Esperando transform desde 'map' a 'base_link'...")
                    rclpy.spin_once(self, timeout_sec=1.0)
            except Exception as e:
                self.get_logger().warn(f"Error al verificar transforms: {e}")
                rclpy.spin_once(self, timeout_sec=1.0)

    def map_callback(self, data):
        """Callback para almacenar los datos del mapa más recientes."""
        self.latest_map = data  # Store the latest map for goal selection
        self.get_logger().debug("Mapa actualizado recibido.")

    def exploration_cycle(self):
        """Ciclo principal de exploración: seleccionar frontiers individuales y navegar hacia ellas."""
        # Si la navegación está en progreso, monitorear
        if self.navigation_in_progress:
            self.monitor_navigation()
            return

        # Si no hay mapa o pose actual, esperar
        if self.latest_map is None:
            self.get_logger().debug("Esperando datos del mapa...")
            return

        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().debug("Esperando pose actual...")
            return

        # Encontrar frontiers
        frontier_indices = self.find_frontiers(self.latest_map)
        if frontier_indices.size == 0:
            self.get_logger().info("No se encontraron frontiers. Exploración completa o se necesita actualizar el mapa.")
            return

        # Convertir índices de frontiers a coordenadas
        frontier_coords = self.get_frontier_coords(frontier_indices)
        if frontier_coords.size == 0:
            self.get_logger().debug("No se pudieron obtener las coordenadas de las frontiers.")
            return

        # Filtrar frontiers basándose en intentos y visitas
        filtered_frontiers = self.filter_frontiers(frontier_coords)
        if filtered_frontiers.size == 0:
            self.get_logger().info("No hay frontiers no visitadas disponibles después del filtrado.")
            return

        # Publicar frontiers para visualización
        self.publish_frontiers(filtered_frontiers)

        # Seleccionar una frontier
        selected_coord = self.select_frontier(filtered_frontiers, current_pose)
        if selected_coord is None:
            self.get_logger().debug("No se pudo seleccionar una frontier válida.")
            return

        # Calcular la pose del objetivo
        goal_pose = self.calculate_goal_pose(selected_coord, current_pose)
        if goal_pose is None:
            self.get_logger().debug("No se pudo calcular la pose del objetivo.")
            return

        # Validar la posición del objetivo
        if not self.is_goal_position_valid(goal_pose):
            self.get_logger().warn("Posición del objetivo seleccionada es inválida (ocupada o fuera de los límites). Se omite.")
            self.mark_frontier_as_invalid(selected_coord)
            return

        # Enviar el objetivo al navigator
        self.navigator.goToPose(goal_pose)
        self.navigation_in_progress = True
        self.current_goal = selected_coord  # Almacena el objetivo actual
        self.get_logger().info(f"Objetivo enviado al navigator: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}")

    def calculate_goal_pose(self, coord, current_pose):
        """Calcula la pose del objetivo desde la coordenada seleccionada."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = coord[0]
        goal_pose.pose.position.y = coord[1]
        goal_pose.pose.position.z = 0.0

        # Calcular la orientación hacia el objetivo
        dx = coord[0] - current_pose.pose.position.x
        dy = coord[1] - current_pose.pose.position.y
        yaw = math.atan2(dy, dx)
        goal_pose.pose.orientation = self.yaw_to_quaternion(yaw)

        return goal_pose

    def yaw_to_quaternion(self, yaw):
        """Convierte un ángulo yaw a un quaternion."""
        from geometry_msgs.msg import Quaternion
        half_yaw = yaw * 0.5
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half_yaw)
        q.w = math.cos(half_yaw)
        return q

    def monitor_navigation(self):
        """Monitorea el estado del objetivo de navegación actual."""
        if self.navigator.isTaskComplete():
            # Verificar el resultado de la tarea de navegación
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Objetivo alcanzado exitosamente!")
                if self.current_goal is not None:
                    coord = self.current_goal
                    self.visited_frontiers.append(coord)
                    key = self.round_coord(coord)
                    self.get_logger().info(f"Marcada frontier en x: {coord[0]:.2f}, y: {coord[1]:.2f} como visitada.")
                    self.current_goal = None  # Resetear objetivo actual
            elif result == TaskResult.CANCELED:
                self.get_logger().info("Objetivo fue cancelado.")
            elif result == TaskResult.FAILED:
                self.get_logger().warn("Fallo al alcanzar el objetivo.")
                if self.current_goal is not None:
                    coord = self.current_goal
                    self.mark_frontier_as_invalid(coord)
                    self.current_goal = None  # Resetear objetivo actual
            else:
                self.get_logger().warn("Resultado del objetivo desconocido.")

            # Resetear el estado de navegación
            self.navigation_in_progress = False
        else:
            # Opcionalmente, implementar monitoreo adicional si es necesario
            pass

    def stop_robot(self):
        """Detiene el robot publicando un mensaje Twist con velocidades cero."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().debug("Publicó mensaje para detener el robot.")

    def find_frontiers(self, map_data):
        """Encuentra celdas de frontera en el mapa usando operaciones vectorizadas de Numpy."""
        width = map_data.info.width
        height = map_data.info.height
        data = np.array(map_data.data, dtype=np.int8).reshape((height, width))

        # Registrar valores únicos de ocupación para depuración
        unique_values = np.unique(data)
        self.get_logger().debug(f"Valores únicos de ocupación en el mapa: {unique_values}")

        # Crear máscaras booleanas para espacios libres y desconocidos
        free_mask = (data == 0)
        unknown_mask = (data == -1)

        # Crear un kernel para verificar vecinos (conectividad de 8)
        kernel = np.array([[1, 1, 1],
                           [1, 0, 1],
                           [1, 1, 1]], dtype=bool)

        # Aplicar convolución para encontrar fronteras
        neighbor_unknown = convolve2d(unknown_mask.astype(int), kernel, mode='same', boundary='fill', fillvalue=0)
        frontier_mask = free_mask & (neighbor_unknown > 0)

        # Obtener índices de fronteras
        frontier_indices = np.flatnonzero(frontier_mask.ravel())
        self.get_logger().debug(f"Total de frontiers identificadas: {len(frontier_indices)}")
        return frontier_indices

    def get_frontier_coords(self, frontier_indices):
        """Convierte índices de frontiers a coordenadas espaciales."""
        if len(frontier_indices) == 0:
            return np.array([])

        width = self.latest_map.info.width
        resolution = self.latest_map.info.resolution
        origin_x = self.latest_map.info.origin.position.x
        origin_y = self.latest_map.info.origin.position.y

        rows = frontier_indices // width
        cols = frontier_indices % width
        xs = (cols * resolution) + origin_x + resolution / 2.0
        ys = (rows * resolution) + origin_y + resolution / 2.0

        frontier_coords = np.column_stack((xs, ys))
        return frontier_coords

    def publish_frontiers(self, frontier_coords):
        """Publica las frontiers como marcadores para visualización en RViz."""
        marker_array = MarkerArray()
        for i, coord in enumerate(frontier_coords):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers_individuales'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.pose.position.z = 0.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.frontier_pub.publish(marker_array)
        self.get_logger().debug(f"Publicadas {len(frontier_coords)} frontiers para visualización.")

    def filter_frontiers(self, frontier_coords):
        """
        Filtra frontiers que ya han sido visitadas o que han alcanzado el máximo de intentos.
        """
        if not self.visited_frontiers:
            return frontier_coords

        filtered_coords = []
        for coord in frontier_coords:
            key = self.round_coord(coord)
            if not self.is_frontier_visited(coord):
                attempts = self.frontier_selection_attempts.get(key, 0)
                if attempts < self.max_attempts:
                    filtered_coords.append(coord)
                else:
                    self.get_logger().info(f"Frontier en x: {coord[0]:.2f}, y: {coord[1]:.2f} ha alcanzado el máximo de intentos y será excluida.")
            else:
                self.get_logger().info(f"Frontier en x: {coord[0]:.2f}, y: {coord[1]:.2f} ha sido visitada y será excluida.")
        self.get_logger().debug(f"Frontiers después del filtrado: {len(filtered_coords)}")
        return np.array(filtered_coords)

    def is_frontier_visited(self, coord):
        """Verifica si una frontier ha sido visitada."""
        for visited in self.visited_frontiers:
            distance = math.hypot(coord[0] - visited[0], coord[1] - visited[1])
            if distance < self.frontier_visit_threshold:
                return True
        return False

    def select_frontier(self, frontier_coords, current_pose):
        """
        Selecciona la frontier más adecuada basada en la ganancia de información y la distancia.
        """
        if len(frontier_coords) == 0:
            return None

        # Priorizar frontiers que maximizan la ganancia de información y la distancia
        scores = []
        for coord in frontier_coords:
            info_gain = self.information_gain(coord, radius=self.info_radius)
            distance = math.hypot(coord[0] - current_pose.pose.position.x, coord[1] - current_pose.pose.position.y)
            score = info_gain * distance  # Ajuste para priorizar la distancia
            scores.append(score)

        if not scores:
            return None

        # Seleccionar la frontier con la mayor puntuación
        max_score_idx = np.argmax(scores)
        selected_coord = frontier_coords[max_score_idx]

        # Incrementar el contador de intentos para esta frontier
        key = self.round_coord(selected_coord)
        self.frontier_selection_attempts[key] = self.frontier_selection_attempts.get(key, 0) + 1
        self.get_logger().info(f"Selected frontier at x: {selected_coord[0]:.2f}, y: {selected_coord[1]:.2f} with score {scores[max_score_idx]:.2f}. Intentos: {self.frontier_selection_attempts[key]}")

        return selected_coord

    def information_gain(self, coord, radius):
        """Calcula la ganancia de información alrededor de una coordenada dada."""
        map_data = self.latest_map
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y

        # Convertir coordenadas a índices
        center_x = int((coord[0] - origin_x) / resolution)
        center_y = int((coord[1] - origin_y) / resolution)
        radius_cells = int(radius / resolution)

        # Extraer ventana alrededor del punto
        x_min = max(0, center_x - radius_cells)
        x_max = min(width, center_x + radius_cells)
        y_min = max(0, center_y - radius_cells)
        y_max = min(height, center_y + radius_cells)

        submap = np.array(map_data.data, dtype=np.int8).reshape((height, width))[y_min:y_max, x_min:x_max]

        # Contar celdas desconocidas
        unknown_cells = np.sum(submap == -1)
        info_gain = unknown_cells * (resolution ** 2)
        return info_gain

    def is_goal_position_valid(self, goal_pose):
        """
        Valida la posición del objetivo para asegurar que esté en espacio libre y dentro de los límites del mapa.
        """
        map_data = self.latest_map
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y

        # Convertir coordenadas a índices de cuadrícula
        col = int((goal_pose.pose.position.x - origin_x) / resolution)
        row = int((goal_pose.pose.position.y - origin_y) / resolution)

        # Verificar si los índices están dentro de los límites
        if col < 0 or col >= width or row < 0 or row >= height:
            self.get_logger().warn(f"Posición del objetivo ({goal_pose.pose.position.x}, {goal_pose.pose.position.y}) está fuera de los límites del mapa.")
            return False

        index = row * width + col
        cell_value = map_data.data[index]

        if cell_value != 0:
            self.get_logger().warn(f"Posición del objetivo ({goal_pose.pose.position.x}, {goal_pose.pose.position.y}) no está libre (valor de celda: {cell_value}).")
            return False

        return True

    def mark_frontier_as_invalid(self, coord):
        """
        Marca una frontier como inválida incrementando sus intentos para prevenir futuras selecciones.
        """
        key = self.round_coord(coord)
        self.frontier_selection_attempts[key] = self.frontier_selection_attempts.get(key, 0) + 1
        self.get_logger().info(f"Marcada frontier en x: {coord[0]:.2f}, y: {coord[1]:.2f} como inválida. Intentos: {self.frontier_selection_attempts[key]}")

def main(args=None):
    rclpy.init(args=args)
    explore_node = Explore()

    try:
        rclpy.spin(explore_node)
    except KeyboardInterrupt:
        explore_node.get_logger().info('Nodo de exploración interrumpido por el usuario.')
    finally:
        explore_node.navigator.lifecycleShutdown()
        explore_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
