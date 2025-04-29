# Importación de librerías necesarias
import rclpy #Librerìa de ros2 para programar nodos
from rclpy.node import Node
from geometry_msgs.msg import Twist #velocidad lineal y angular de la toruga
from std_srvs.srv import Empty #borrrar pantalla
from turtlesim.srv import TeleportAbsolute #Hace que la tortuga aparezca en un punto especìfico
from turtlesim.msg import Pose #contiene la posiciòn y orientaciòn de la tortuga
import curses #lee entradas del teclado
import math #funciones y variables matemàticas


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller') #Nodo

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) #enviar comando sde velocidad
        self.clear_client = self.create_client(Empty, '/clear') #limpiar pantalla
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute') #teletransportar tortuga
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10) #recibe la poisicòn de la tortuga

        while not self.clear_client.wait_for_service(timeout_sec=1.0): #esperar que cada funciòn se pueda usar
            self.get_logger().info('Esperando al servicio /clear...')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /teleport_absolute...')

        self.timer = None
        self.stage = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def pose_callback(self, pose_message): #actualiza la posiciòn de la tortuga
        self.x = pose_message.x
        self.y = pose_message.y
        self.theta = pose_message.theta

    def clear_trail(self):
        req = Empty.Request() #Limpiar pantalla
        self.clear_client.call_async(req)
        self.get_logger().info('¡Trayectoria limpiada!')

    def teleport_to_start(self, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x = x #Teletransporta la tortuga a la direcciòn (x,y)
        req.y = y
        req.theta = theta
        self.teleport_client.call_async(req)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi: #hac eque el àngulo estè en un rango entre pi y -pi
            angle += 2 * math.pi
        return angle

    def start_drawing(self, letter):
        self.clear_trail() #limpia la pantalla
        self.teleport_to_start(5.544445, 5.544445, 0.0) #posiciòn inicial
        self.letter = letter #almacena la letra
        self.stage = f'start_{letter.lower()}' #estado inicial
        if self.timer:
            self.timer.cancel()
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self): #Bucle de control
        msg = Twist()

        if self.letter == 'M':
            self.draw_M(msg)
        elif self.letter == 'J':
            self.draw_J(msg)
        elif self.letter == 'L':
            self.draw_L(msg)
        elif self.letter == 'R':
            self.draw_R(msg)

        self.publisher_.publish(msg) #mensaje de velocidad de la otrtuga

    # =========================
    # ==== LÓGICAS DE LETRAS ===
    # =========================

    # M
    def draw_M(self, msg):
        if self.stage == 'start_m':
            self.initial_x = self.x
            self.initial_y = self.y
            self.stage = 'rotate_up_m'
        elif self.stage == 'rotate_up_m':
            if abs(self.normalize_angle(self.theta - math.pi/2)) > 0.05:
                msg.angular.z = 4.0
            else:
                self.stage = 'move_up_m'
        elif self.stage == 'move_up_m':
            if abs(self.y - self.initial_y) < 2.0:
                msg.linear.x = 6.0
            else:
                self.stage = 'rotate_diag_right_m'
        elif self.stage == 'rotate_diag_right_m':
            if abs(self.normalize_angle(self.theta + math.pi/4)) > 0.05:
                msg.angular.z = -4.0
            else:
                self.stage = 'move_diag_right_m'
                self.initial_x = self.x
                self.initial_y = self.y
        elif self.stage == 'move_diag_right_m':
            if math.hypot(self.x - self.initial_x, self.y - self.initial_y) < 1.5:
                msg.linear.x = 6.0
            else:
                self.stage = 'rotate_diag_left_m'
        elif self.stage == 'rotate_diag_left_m':
            if abs(self.normalize_angle(self.theta - math.pi/4)) > 0.05:
                msg.angular.z = 4.0
            else:
                self.stage = 'move_diag_left_m'
                self.initial_x = self.x
                self.initial_y = self.y
        elif self.stage == 'move_diag_left_m':
            if math.hypot(self.x - self.initial_x, self.y - self.initial_y) < 1.5:
                msg.linear.x =6.0
            else:
                self.stage = 'rotate_down_m'
        elif self.stage == 'rotate_down_m':
            if abs(self.normalize_angle(self.theta + math.pi/2)) > 0.05:
                msg.angular.z = -4.0
            else:
                self.stage = 'move_down_m'
                self.initial_y = self.y
        elif self.stage == 'move_down_m':
            if abs(self.y - self.initial_y) < 2.0:
                msg.linear.x = 6.0
            else:
                self.finish_letter()

    # J
    def draw_J(self, msg):
        if self.stage == 'start_j':
            self.initial_y = self.y
            self.stage = 'rotate_down_j'
        elif self.stage == 'rotate_down_j':
            if abs(self.normalize_angle(self.theta + math.pi/2)) > 0.05:
                msg.angular.z = 4.0
            else:
                self.stage = 'move_down_j'
        elif self.stage == 'move_down_j':
            if abs(self.y - self.initial_y) < 2.0:
                msg.linear.x = 6.0
            else:
                self.stage = 'curve_right_j'
                self.initial_theta = self.theta
        elif self.stage == 'curve_right_j':
            msg.linear.x = 4.5
            msg.angular.z = -4.5
            delta_theta = self.normalize_angle(self.theta - self.initial_theta)
            if abs(delta_theta) > math.pi/3:
                self.finish_letter()

    # L
    def draw_L(self, msg):
        if self.stage == 'start_l':
            self.initial_y = self.y
            self.stage = 'rotate_down_l'
        elif self.stage == 'rotate_down_l':
            if abs(self.normalize_angle(self.theta + math.pi/2)) > 0.05:
                msg.angular.z = -4.0
            else:
                self.stage = 'move_down_l'
        elif self.stage == 'move_down_l':
            if abs(self.y - self.initial_y) < 3.0:
                msg.linear.x = 6.0
            else:
                self.stage = 'rotate_right_l'
        elif self.stage == 'rotate_right_l':
            if abs(self.normalize_angle(self.theta)) > 0.05:
                msg.angular.z = 4.0
            else:
                self.stage = 'move_right_l'
                self.initial_x = self.x
        elif self.stage == 'move_right_l':
            if abs(self.x - self.initial_x) < 1.5:
                msg.linear.x = 6.0
            else:
                self.finish_letter()

        # R
    def draw_R(self, msg):
        if self.stage == 'start_r':
            self.initial_y = self.y
            self.stage = 'rotate_up_r'
        elif self.stage == 'rotate_up_r':
            if abs(self.normalize_angle(self.theta - math.pi/2)) > 0.05:
                msg.angular.z = 4.0
            else:
                self.stage = 'move_up_r'
                self.initial_y = self.y
        elif self.stage == 'move_up_r':
            if abs(self.y - self.initial_y) < 3.0:
                msg.linear.x = 6.0
            else:
                self.stage = 'rotate_right_r'
        elif self.stage == 'rotate_right_r':
            if abs(self.normalize_angle(self.theta)) > 0.05:
                msg.angular.z = -4.0
            else:
                self.stage = 'move_right_r'
                self.initial_x = self.x
        elif self.stage == 'move_right_r':
            if abs(self.x - self.initial_x) < 1.5:
                msg.linear.x = 6.0
            else:
                self.stage = 'rotate_down_r'
        elif self.stage == 'rotate_down_r':
            if abs(self.normalize_angle(self.theta + math.pi/2)) > 0.05:
                msg.angular.z = -4.0
            else:
                self.stage = 'move_down_r'
                self.initial_y = self.y
        elif self.stage == 'move_down_r':
            if abs(self.y - self.initial_y) < 1.5:  # Ajustar la altura de la parte inferior de la R
                msg.linear.x = 6.0
            else:
                self.stage = 'rotate_left_r'  # Giro a la izquierda
        elif self.stage == 'rotate_left_r':
            if abs(self.normalize_angle(self.theta - math.pi)) > 0.05:
                msg.angular.z = 4.0# Girar a la izquierda
            else:
                self.stage = 'move_left_r'  # Movimiento corto hacia la izquierda
                self.initial_x = self.x
        elif self.stage == 'move_left_r':
            if abs(self.x - self.initial_x) < 1.8 : #movimiento a la izquierda
                msg.linear.x = 6.0
            else:
                self.stage = 'rotate_right_short_r'  # Giro corto hacia la derecha
        elif self.stage == 'rotate_right_short_r':
            if abs(self.normalize_angle(self.theta)) > 0.05:
                msg.angular.z = -4.0   #Giro corto a la derecha
            else:
                self.stage = 'move_right_short_r'  # Movimiento corto hacia la derecha
                self.initial_x = self.x
        elif self.stage == 'move_right_short_r':
            if abs(self.x - self.initial_x) < 1.0:  # Movimiento corto hacia la derecha
                msg.linear.x = 6.0
            else:
                self.stage = 'rotate_diag_down_right_r'  # Giro para la diagonal hacia abajo
        elif self.stage == 'rotate_diag_down_right_r':
            if abs(self.normalize_angle(self.theta + math.pi/4)) > 0.05:
                msg.angular.z = -4.0   #Giro hacia la diagonal
            else:
                self.stage = 'move_diag_down_right_r'
                self.initial_x = self.x
                self.initial_y = self.y   # <-- GUARDAR también el Y AQUÍ
        elif self.stage == 'move_diag_down_right_r':
             # Movimiento hacia abajo en la diagonal
            if math.hypot(self.x - self.initial_x, self.y - self.initial_y) < 2.0: 
             msg.linear.x = 6.0
            else:
             self.finish_letter()



    def finish_letter(self):
        self.get_logger().info(f'¡Letra {self.letter} dibujada correctamente!') #mensaje letra dibujada
        if self.timer:
            self.timer.cancel()
        self.stage = None

    # =========================
    # === LOOP DE TECLADO ======
    # =========================
    def control_loop_keyboard(self, stdscr): #lectura teclado
        curses.cbreak()
        stdscr.nodelay(True)
        stdscr.clear()
        stdscr.addstr(0, 0, "Flechas para mover, M/J/L/R para dibujar, X limpiar, Q salir") #instrucciones
        stdscr.refresh()

        while rclpy.ok():
            key = stdscr.getch()
            msg = Twist()

            if key == curses.KEY_UP:
                msg.linear.x = 2.0
            elif key == curses.KEY_DOWN:
                msg.linear.x = -2.0
            elif key == curses.KEY_LEFT:
                msg.angular.z = 2.0
            elif key == curses.KEY_RIGHT:
                msg.angular.z = -2.0
            elif key == ord('m'):
                self.start_drawing('M')
            elif key == ord('j'):
                self.start_drawing('J')
            elif key == ord('l'):
                self.start_drawing('L')
            elif key == ord('r'):
                self.start_drawing('R')
            elif key == ord('x'):
                self.clear_trail()
            elif key == ord('q'):
                break

            self.publisher_.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

# Función principal
def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        curses.wrapper(node.control_loop_keyboard) #funcion lectura de teclado
    finally:
        node.destroy_node()
        rclpy.shutdown()
