# ROS 2 Practices for Robotics Master’s Program

## [Práctica 1: Introducción a ROS 2](./practica_1/Readme_1.md)
- **Objetivo:**
Configurar un espacio de trabajo de ROS 2 y familiarizarse con conceptos básicos de ROS como publicar/suscribirse, creación de nodos y compilación con colcon.
- **Tareas Clave:**
  - Configuración del entorno de ROS 2 en Ubuntu 22.04.
  - Creación de un ejemplo simple de publicación/suscripción en Python y C++.
  - Uso de `rosdep` para instalar dependencias.
  - Comprensión de la estructura del espacio de trabajo de ROS 2.
  - Empleo de usb_cam para visualizar la cam.
- **Herramientas:** 
  - ROS 2 Humble
  - Image_view

## [Práctica 2: Simulando Turtlebot 4 en Gazebo Ignition](./practica_2/Readme_2.md)
- **Objetivo:**
Simular Turtlebot 4 en Gazebo Ignition y controlarlo utilizando ROS 2.
- **Tareas Clave**
  - Instalar Gazebo Ignition y los paquetes de simulación de Turtlebot 4.
  - Ejecutar la simulación y controlar Turtlebot 4 mediante temas de ROS 2.
  - Personalizar el mundo de la simulación y visualizar los datos de los sensores en RViz.
- **Herramientas:**
  - Gazebo Ignition versión Fortrees
  - RViz
  - ROS 2 Humble
  - Turtlebot4

## [Práctica 3: Integración de Gazebo Fortress](./practica_3/readme_3.md)
- **Objetivo**: Trabajar con el simulador moderno Gazebo (Ignition Fortress) e integrarlo con ROS 2 para simulación. Además, definir un nuevo un *workspace* con Turtlebot3.
- **Tareas clave**:
  - Intslar Turtlebot3.
  - Instalar y configurar Gazebo Ignition Fortress.
  - Ejecutar comandos básicos de Gazebo e integrarlos con ROS 2.
  - Comprender el formato de descripción de simulación (SDF) y usarlo para modelar robots.
- **Herramientas**: Gazebo Ignition Fortress, Docker o Ubuntu nativo (mi situación), ROS 2 Humble, Turtlebot 3, Nav2.

## [Práctica 4: Creación de Mapas con SLAM Toolbox](./practica_4/readme_4.md)
- **Objetivo**: Generar mapas en ROS 2 utilizando SLAM Toolbox.
- **Tareas clave**:
  - Configurar la simulación de Turtlebot 3 en Gazebo.
  - Usar SLAM Toolbox para crear y guardar mapas.
  - Explorar la navegación y la evitación de obstáculos.
- **Herramientas**: Gazebo, ROS 2 Humble,Navigation 2, SLAM Toolbox.

## [Práctica 5: Navegación con Nav2 Simple Commander](./practica_5/readme_5.md)
- **Objetivo**: Controlar la navegación de Turtlebot 3 utilizando Nav2 Simple Commander en ROS 2.
- **Tareas clave**:
  - Familiarizarse con la instalación y configuración de los paquetes de navegación en ROS 2.
  - Configurar un entorno de simulación con Turtlebot3 en Gazebo.
  - Enviar objetivos de navegación al robot mediante Nav2 Simple Commander.
  - Modificar parámetros en los archivos de configuración de Nav2 para personalizar el comportamiento de navegación.
- **Herramientas**: ROS 2 Humble, Nav2, Turtlebot 3, Gazebo.

## [Práctica 6: Integración de un Chatbot en ROS 2](./practica_6/readme_6.md)
- **Objetivo**: Integrar un chatbot con ROS 2 para mejorar la interacción con el robot.
- **Tareas clave**:
  - Configurar y ajustar el chatbot en ROS 2.
  - Probar la funcionalidad del chatbot y ampliarla.
  - Usar el chatbot para interacciones conversacionales con el robot.
- **Herramientas**: ROS 2 Humble, Chatbot, RASA.

## [Proyecto Final: Navegación Avanzada con Interacción por Voz](./Final_proyect/readme_final.md)
- **Objetivo**: Desarrollar un sistema avanzado de navegación que integre comandos de voz y múltiples planificadores locales.
- **Tareas clave**:
  - Leer y navegar hacia waypoints desde un archivo.
  - Implementar interacción por voz para proporcionar retroalimentación sobre la navegación.
  - Cambiar dinámicamente los planificadores de navegación según factores ambientales.
- **Herramientas**: ROS 2 Humble, Nav2, Reconocimiento de Voz, TTS (Texto a Voz).