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
 
Dentro del **workspace** creado en clase (my_turtle_controller), se procedió a editar el archivo move_turtle.py para poder mover la tortuga utilizando las flechas del teclado. Como primer paso, se agregaron nuevas librerías además de las que ya existían. Las librerías incorporadas fueron:

◦ Empty: Esta librería es un tipo especial de servicio definido en el paquete std_srvs, que se usa cuando se quiere activar una acción en un nodo sin tener que enviarle parámetros.Un ejemplo común de su uso es la limpieza de la pantalla en una simulación.

◦ curses: Esta librería proporciona un conjunto de funciones para la gestión de entradas de teclado permitiendo capturar pulsaciones de teclas sin necesidad de presionar "Enter" y gestionar eventos de teclado en tiempo real.

La libreria Empty fue utilizada para implementar una funciòn adicional que permita limpiar los trazos de las trayectorias marcadas en la pantalla de turtlesim.
 
 ```
#Importaciòn de librerias

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty # Libreria adicional para limpiar pantalla
import curses # Libreria adicional para permitir la entrada de teclado
 
```
# Diseño y funcionamiento
