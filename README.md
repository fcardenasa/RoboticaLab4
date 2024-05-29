# Laboratorio #4 de robótica. Cinemática Directa - Phantom X - ROS
Nombre: Fernando Cardenas Acosta & Raúl Ignacio Marín Medina

## Objetivos: 
• Crear todos los Joint Controllers con ROS para manipular servomotores Dynamixel AX-12 del robot Phantom X Pincher.

• Manipular los tópicos de estado y comando para todos los Joint Controllers del robot Phantom X Pincher.

• Manipular los servicios para todos los Joint Controllers del robot Phantom X Pincher.

• Conectar el robot Phantom X Pincher con MATLAB o Python usando ROS.

## Requisitos:
• Ubuntu versión 20.xx preferible 20.04 LTS con ROS.

• Espacio de trabajo para catkin correctamente configurado.

• Paquetes de Dynamixel Workbench. https://github.com/fegonzalez7/rob_unal_clase3

• Paquete del robot Phantom X: https://github.com/felipeg17/px_robot.

• Python o MATLAB 2015b o superior instalado en el equipo.

• Robotics toolbox de Mathworks (Disponible desde la versión 2015 en adelante).

• Toolbox de robótica de Peter Corke.

• Un (1) manipulador Phantom X Pincher con su base en madera.

## Desarrollo.

El laboratorio se llevó a cabo dese dos frentes, el primero, la programación en ROS del controlador para el Phantom, cuyas evidencias incluyen el video de funcionamiento del robot, el codigo de python y la seleccion de las opciones. El segundo es el desarrollo del programa de matlab en el que se muestra la configuración del robot.

## PHANTOM

# Video de funcionamiento.

https://youtu.be/d_em7HzUAmo

# Imagenes de la selección de opciones.

Imagen de la bienvenida y selección de posición del usuario.

![image](https://github.com/fcardenasa/RoboticaLab4/blob/main/fotoInicio.jpg)

Imagen de las ángulos y errores medidos por el programa.

![image](https://github.com/fcardenasa/RoboticaLab4/blob/main/fotoMedidas.jpg)

Imagen del mensaje de error entregado al no ser posible la conexión con el robot.

![image](https://github.com/fcardenasa/RoboticaLab4/blob/main/fotoError.jpg)

Imagen de la posición de home.

![image](https://github.com/fcardenasa/RoboticaLab4/blob/main/foto1.jpg)

Imagen de la posición 1.

![image](https://github.com/fcardenasa/RoboticaLab4/blob/main/foto2.jpg)

Imagen de la posición 2.

![image](https://github.com/fcardenasa/RoboticaLab4/blob/main/foto3.jpg)

Imagen de la posición 3.

![image](https://github.com/fcardenasa/RoboticaLab4/blob/main/foto4.jpg)
Imagen de la posición 4.

![image](https://github.com/fcardenasa/RoboticaLab4/blob/main/foto5.jpg)

## MATLAB

En esta sección del laboratorio, se plantea la matriz DH partiendo de la cinemática inversa del Phantom X Pincher. A continuación, se presentan la tabla de parámetros DH obtenidos según la convención NOA:

![WhatsApp Image 2024-05-28 at 22 23 37_6f9ce45e]

(https://github.com/fcardenasa/RoboticaLab4/assets/124843458/5a6554f6-8603-425c-8745-843383b19c88)
