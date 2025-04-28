# ********************************************* Control de movimiento manual**********************************************************

# Importación de librerías necesarias
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist          
from std_srvs.srv import Empty              # Librería adicional para limpiar la pantalla
import curses                               # Librería adicional para permitir la entrada de teclado

# Creación de la clase TurtleController

class TurtleController(Node):
    def __init__(self):			   						
        super().__init__('turtle_controller')                				            # Inicialización del nodo con el nombre 'turtle_controller'
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)          # Creación de un publicador para enviar comandos de movimiento a '/turtle1/cmd_vel'
        self.clear_client = self.create_client(Empty, '/clear')				            # Cliente para llamar al servicio de limpiar la trayectoria de la tortuga
        
        
        while not self.clear_client.wait_for_service(timeout_sec=1.0):			        # Espera hasta que el servicio de limpiar la trayectoria esté disponible
            self.get_logger().info('Esperando al servicio /clear...')

    # Función que llama al servicio para limpiar la trayectoria de la tortuga
    def clear_trail(self):
        req = Empty.Request() 								     # Petición vacía, ya que el servicio no requiere parámetros
        self.clear_client.call_async(req) 						 # Llamada asincrónica al servicio
        self.get_logger().info('¡Trayectoria limpiada!')

    # Bucle principal que gestiona la entrada del teclado y envía los comandos a la tortuga
    def control_loop(self, stdscr):
   
	# Configuración de curses para la lectura del teclado sin bloqueo
        curses.cbreak()
        stdscr.nodelay(True)
        stdscr.clear()
        
        # Mensaje de instrucciones que se muestra en la pantalla
        stdscr.addstr(0, 0, "↑ ↓ ← → para mover, 'c' limpiar trayectoria, 'q' salir.")
        stdscr.refresh()

        # Bucle de control que continuará hasta que se cierre el nodo o se presione 'q'
        while rclpy.ok():
            key = stdscr.getch()							 # Captura la tecla presionada
            msg = Twist()  								     # Crea un nuevo mensaje de tipo Twist para enviar comandos

            # Condiciones para mover la tortuga dependiendo de la tecla presionada
            if key == curses.KEY_UP:
                msg.linear.x = 2.0 							 # Mover hacia adelante
            elif key == curses.KEY_DOWN:
                msg.linear.x = -2.0 						 # Mover hacia atrás
            elif key == curses.KEY_LEFT:
                msg.angular.z = 2.0							 # Girar a la izquierda
            elif key == curses.KEY_RIGHT:
                msg.angular.z = -2.0  						 # Girar a la derecha
            elif key == ord('t'):
                self.clear_trail()  						 # Llamar al servicio para limpiar la trayectoria
            elif key == ord('q'):
                break 									     # Salir del bucle y cerrar el programa

            
            self.publisher_.publish(msg)					 # Publicar el mensaje con el movimiento
            rclpy.spin_once(self, timeout_sec=0.1)	     	 # Realizar el ciclo de procesamiento de ROS

# Función principal que inicializa y ejecuta el nodo
def main(args=None):
    rclpy.init(args=args) 							    	 # Inicializar el sistema ROS 2
    node = TurtleController()								 # Crear el nodo de control de la tortuga    
    try:       
        curses.wrapper(node.control_loop)                   # Ejecutar el bucle de control dentro del entorno curses
    finally:
        node.destroy_node()  								# Destruir el nodo al finalizar
        rclpy.shutdown()  								    # Apagar ROS 2