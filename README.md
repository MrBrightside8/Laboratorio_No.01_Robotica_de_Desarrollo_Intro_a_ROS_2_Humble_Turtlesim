# Laboratorio No. 01 Robótica de Desarrollo Intro a ROS 2 Humble Turtlesim

## Planteamiento del problema

Dentro del **workspace** creado en clase (*my_turtle_controller*), se debe editar el archivo **move turtle.py**, el objetivo es controlar el movimiento de la tortuga en el simulador *turtlesim* mediante el teclado, cumpliendo con los siguientes requerimientos:

## Objetivos

1. Permitir mover la tortuga de forma lineal y angular utilizando las flechas del teclado: <br>

  Acciones asignadas:<br>
◦ Flecha ↑: avanzar hacia adelante.<br>
◦ Flecha ↓: retroceder.<br>
◦ Flecha ←: girar a la izquierda.<br>
◦ Flecha →: girar a la derecha.<br>

## Procedimiento

 **1. Control de movimiento manual**
 
 ```
#Importaciòn de librerias

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
 
```
# Diseño y funcionamiento
