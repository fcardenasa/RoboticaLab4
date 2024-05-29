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

A partir de estos datos, se usa el toolboz de robótica SerialLink para crear el robot con parámetros DH. Los eslabones se crean a partir de la función link(theta,d,a,alpha), se le puede agregar dos parámetros adicionales, los cuales serán link(theta,d,a,alpha,1/0,offset) donde 1 será para articulación prismática mientras que 0 será para articulación rotacional. De igual manera, se establecen los límites espaciales de las juntas mediante la función qlim después de definir cada sistema de coordenadas. para este caso, se plantea la restricción entre [-pi,pi]. Por último, se puede definir la posición de la base del robot al usar la palabra 'base' dentro de la función SerialLink(), adicionalmente, se plantea la matriz de rotación desde el TCP con convención NOA hasta el sistema de coordenada de la última articulación para tener la configuración correcta de ejes.

* La función phantom.plot() permite visualizar el robot phantom en la bse definida para el robot, mientras que la función phantom.teach() servirá para interactuar con el controlador del toolbox e indicar los parámetros q para que el robot se mueva en el espacio. A continuación, se anexa el código 

![image](https://github.com/fcardenasa/RoboticaLab4/assets/124843458/9a9633c1-b42f-4dc1-afed-662f231ebe94)

![image](https://github.com/fcardenasa/RoboticaLab4/assets/124843458/7772a639-9433-4383-aa2a-e0dd744cc19e)

![image](https://github.com/fcardenasa/RoboticaLab4/assets/124843458/d8cdd838-9b70-44f8-8870-0f2f946251c7)

...

clf;
%PLOTEAR ROBOT Y CADENA CINEMÁTICA CON DH PARAMETERS STD
%if prismatic joint theta=theta, d=0, offset=1, poner valor de d después de
%offset
%if revolute joint: theta=0, offset=0, después valor de theta
%L(n)=Link([theta d r/a alpha])
%PLOTEAR PARA ROBOT 4GDL
L1=Link([0 100 0 pi/2 0 0]);
L1.qlim=[-pi pi];
L2=Link([0 0 104.8 0 0 0]);
L2.qlim=[-pi pi];
L3=Link([0 0 104.8 0 0 0]);
L3.qlim=[-pi pi];
L4=Link([0 0 75.76 0 0 0]);
L4.qlim=[-pi pi];
%%Parámetros para SerialLink
World=eye(4,4); %%Misma orientación con posición (0,0,0)
Tool=[0 0 1 0; 1 0 0 0; 0 1 0 0; 0 0 0 1]; %Matriz Homog. herramienta
Rob=SerialLink([L1 L2 L3 L4],'name',"PhantomII",'tool',Tool,'base',World)
home=[0,0,0,0];
Rob.plot(home); %%Interfaz gráfica del robot
Rob.teach() %%Saca las variables de cada articulación para enseñar

%MATRIZ HOMOGÉNEA DESDE TCP HASTA BASE
Htcp04=DH(0,-90,75.76,0);
H0403=DH(0,0,104.8,0);
H0302=DH(0,0,104.8,0);
H0201=DH(0,-90,0,100);
base=eye(4,4)*H0201*H0302*H0403*Htcp04;

%%ENSEÑAR AL ROBOT POSICIONES
Rob.plot([0,pi/4,-pi/4,0])

...

(https://github.com/fcardenasa/RoboticaLab4/assets/124843458/5a6554f6-8603-425c-8745-843383b19c88)
