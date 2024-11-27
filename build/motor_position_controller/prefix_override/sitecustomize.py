import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ajdj/Workspaces/turtlebot3_ws/turtlebot3_modulo2/install/motor_position_controller'
