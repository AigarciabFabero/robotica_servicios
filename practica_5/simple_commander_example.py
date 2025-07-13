from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
import time

def main():
    # Inicializa el sistema ROS 2
    rclpy.init()

    # Crea una instancia del navegador básico
    navigator = BasicNavigator()

    # Configura la meta del robot
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'  # Marco de referencia
    goal_pose.pose.position.x = 1.65    # Coordenada X de la meta
    goal_pose.pose.position.y = 0.55    # Coordenada Y de la meta
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.w = 1.0  # Orientación (cuaternión simplificado)

    # Envía al robot hacia la posición objetivo
    navigator.goToPose(goal_pose)

    # Monitorea el progreso hacia el objetivo
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Distancia restante al objetivo: {feedback.distance_remaining:.2f} metros")
        time.sleep(1)  # Espera un segundo antes de verificar nuevamente

    # Verifica el resultado final de la navegación
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("El robot alcanzó el objetivo.")
    elif result == TaskResult.CANCELED:
        print("La navegación fue cancelada.")
    else:
        print("La navegación falló.")

    # Finaliza el sistema ROS 2
    rclpy.shutdown()

# Punto de entrada principal
if __name__ == '__main__':
    main()
