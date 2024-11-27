import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def create_corrected_flowchart():
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.axis('off')

    # Define box properties
    box_props = dict(boxstyle="round,pad=0.3", edgecolor="black", facecolor="lightblue")

    # Define arrow properties
    arrow_props = dict(arrowstyle="->", color="black", lw=1.5)

    # Add the flowchart boxes
    positions = [
        (5, 9, "motor_position_controller\npublica en /motor1_position\ny /motor2_position"),
        (5, 6.5, "joint_state_publisher_custom\ntransforma posiciones\na JointState y publica en /joint_states"),
        (5, 4, "robot_state_publisher\nactualiza transformaciones\nen el sistema de coordenadas"),
        (5, 1.5, "ESP32 con Micro-ROS\nrecibe posiciones y mueve los motores")
    ]

    boxes = [ax.text(x, y, text, ha="center", va="center", bbox=box_props) for (x, y, text) in positions]

    # Add arrows (corrected direction: top to bottom)
    for i in range(len(positions) - 1):
        ax.annotate("", xy=(5, positions[i][1] - 2.2), xytext=(5, positions[i][1] - 0.6), arrowprops=arrow_props)

    plt.title("Flujo de Datos", fontsize=16, fontweight='bold', pad=20)
    plt.show()

create_corrected_flowchart()
