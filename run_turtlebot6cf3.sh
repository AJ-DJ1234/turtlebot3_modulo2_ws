#!/bin/bash

# Función para terminar los procesos de ROS2
cleanup() {
    echo ""
    echo "Terminating ROS2 processes..."
    echo ""
    pkill -f ros2
    exit 0
}

# Capturar la señal de interrupción (Ctrl+C)
trap cleanup SIGINT

# ------------------------------------------
# Lanza el nodo de SLAM Toolbox
echo ""
echo "Launching SLAM Toolbox..."
echo ""
gnome-terminal -- bash -c "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false; exec bash" &
# Espera 5 segundos para asegurar que el nodo se inicie correctamente
sleep 5

# ------------------------------------------
# Lanza el nodo de navegación de TurtleBot3
echo ""
echo "Launching TurtleBot3 navigation..."
echo ""
gnome-terminal -- bash -c "ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=false use_amcl:=true" &
# Espera 5 segundos para asegurar que el nodo se inicie correctamente
sleep 5

# ------------------------------------------
# Fuente del archivo de configuración de ROS2
echo ""
echo "Sourcing setup.bash..."
echo ""
. install/setup.bash

# ------------------------------------------
# Ejecuta el nodo de marcadores de hazmat
echo ""
echo "Running hazmat marker node..."
echo ""
gnome-terminal -- bash -c "cd /home/ajdj/Workspaces/turtlebot3_ws/turtlebot3_module2/; . install/setup.bash; ros2 run hazmat_marker hazmat_marker_aj; exec bash" &
# Espera 5 segundos para asegurar que el nodo se inicie correctamente
sleep 5

# ------------------------------------------
# Ejecuta el servidor de la cámara
echo ""
echo "Running camera server..."
echo ""
gnome-terminal -- bash -c "cd /home/ajdj/Workspaces/turtlebot3_ws/turtlebot3_module2/; . install/setup.bash; ros2 run camara camara_server; exec bash" &
# Espera 5 segundos para asegurar que el nodo se inicie correctamente
sleep 5

# ------------------------------------------
# Ejecuta el nodo de la interfaz
echo ""
echo "Running map saver service..."
echo ""
ros2 run interface_turtlebot map_saver_service &
# Espera 5 segundos para asegurar que el servicio se inicie correctamente
sleep 5

# ------------------------------------------
# Ejecuta el nodo de la interfaz
echo ""
echo "Running interface..."
echo ""
ros2 run interface_turtlebot interface_final2 &
# Espera 5 segundos para asegurar que el nodo se inicie correctamente
sleep 5

# ------------------------------------------
echo ""
echo ""
echo "All commands executed successfully."
echo ""
echo ""

# Esperar indefinidamente hasta que se reciba una señal de interrupción
while true; do
    sleep 1
done
