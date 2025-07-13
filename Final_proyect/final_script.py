from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
import time
import yaml
import random
import subprocess
import threading
import speech_recognition as sr
import math
import os

def read_waypoints(yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
        waypoints = data.get('waypoints', [])
    return waypoints

def synthesize_speech(message):
    """
    Utiliza espeak para la síntesis de voz.
    Asegúrate de tener espeak instalado: sudo apt-get install espeak
    """
    subprocess.call(['espeak', '-v', 'es', '-s', '135', message])

def recognize_command():
    """
    Utiliza la librería speech_recognition para reconocer comandos de voz.
    Asegúrate de tener instaladas las dependencias:
    pip install SpeechRecognition pyaudio
    """
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("Escuchando comando de voz...")
        audio = recognizer.listen(source)
    try:
        command = recognizer.recognize_google(audio, language='es-ES')
        print(f"Comando reconocido: {command}")
        return command.lower()
    except sr.UnknownValueError:
        print("No se entendió el comando.")
        return ""
    except sr.RequestError as e:
        print(f"Error con el servicio de reconocimiento de voz; {e}")
        return ""

def navigate_to_waypoint(navigator, waypoint, choice):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'  # Marco de referencia
    goal_pose.pose.position.x = waypoint['x']
    goal_pose.pose.position.y = waypoint['y']
    
    # Convertir theta a cuaternión
    theta = waypoint['theta']
    goal_pose.pose.orientation.z = math.sin(theta / 2)
    goal_pose.pose.orientation.w = math.cos(theta / 2)

    navigator.goToPose(goal_pose)

    # Monitorea el progreso hacia el objetivo
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Distancia restante al objetivo: {feedback.distance_remaining:.2f} metros")
        time.sleep(1)  

    # Verifica el resultado final de la navegación
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("El robot alcanzó el objetivo.")
        if (choice == '2'):
            synthesize_speech("He alcanzado el objetivo.")
        return True
    elif result == TaskResult.CANCELED:
        print("La navegación fue cancelada.")
        if (choice == '2'):
            synthesize_speech("La navegación fue cancelada.")
        return False
    else:
        print("La navegación falló.")
        if (choice == '2'):
            synthesize_speech("La navegación falló.")
        return False

def main():
    # Inicializa el sistema ROS 2
    rclpy.init()

    # Crea una instancia del navegador básico
    navigator = BasicNavigator()

    # Opciones de práctica
    print("Seleccione la opción de práctica:")
    print("1. Práctica 5")
    print("2. Práctica Final")
    choice = input("Ingrese 1 o 2: ")

    if choice == '1':
        # Práctica 5
        waypoint = {'x': 1.70,'y': 0.55,'theta': 1.0}
        success = navigate_to_waypoint(navigator, waypoint, choice)

    elif choice == '2':
        # Práctica final
        waypoints_path = os.path.join(os.path.dirname(__file__), 'waypoints.yaml')
        waypoints = read_waypoints(waypoints_path)
        if not waypoints:
            print("No se encontraron waypoints en el archivo.")
            rclpy.shutdown()
            return

        print("¿Cómo desea enviar los waypoints?")
        print("1. Secuencialmente")
        print("2. Aleatoriamente")
        send_choice = input("Ingrese 1 o 2: ")

        if send_choice == '1':
            waypoint_order = waypoints
        elif send_choice == '2':
            waypoint_order = waypoints.copy()
            random.shuffle(waypoint_order)
        else:
            print("Opción inválida. Saliendo.")
            rclpy.shutdown()
            return

        for idx, waypoint in enumerate(waypoint_order):
            print(f"Navegando al waypoint {idx + 1}: {waypoint}")
            success = navigate_to_waypoint(navigator, waypoint, choice)
            if not success:
                print("Navegación fallida o cancelada. Deteniendo.")
                break

            synthesize_speech("¿Desea continuar a la siguiente posición? Di sí para continuar o detener para finalizar la navegación.")
            
            # Espera el comando de voz en un hilo separado para no bloquear
            command = ""

            def get_command():
                nonlocal command
                command = recognize_command()

            command_thread = threading.Thread(target=get_command)
            command_thread.start()
            command_thread.join(timeout=10)  # Espera hasta 10 segundos por un comando

            if command.startswith("sí") or command.startswith("si"):
                continue
            elif "detener" in command:
                print("Deteniendo la navegación según comando del usuario.")
                synthesize_speech("Deteniendo la navegación.")
                navigator.cancelAll()
                break
            else:
                print("Comando no reconocido. Continuando por defecto.")
                synthesize_speech("Continuando por defecto.")
                continue

    else:
        print("Opción inválida. Saliendo.")

    # Finaliza el sistema ROS 2
    rclpy.shutdown()

if __name__ == '__main__':
    main()