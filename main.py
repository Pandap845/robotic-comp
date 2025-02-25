#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, InfraredSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxClient, TextMailbox, NumericMailbox, LogicMailbox
from enum import Enum
import math as m
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop


#---------------------------------------
# CONSTANTES
#---------------------------------------

MAX_SPEED = 1000;
WHEEL_DIAMETER = 33.5;
DETECT_DISTANT = 22.5;


# -----------------------------------------
# Inicialización de los sensores y motores
# -----------------------------------------
ev3 = EV3Brick()
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
claw = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
c_sensor1 = ColorSensor(Port.S4)
c_sensor2 = ColorSensor(Port.S3)
c_sensor3 = ColorSensor(Port.S1)
infra_sensor = InfraredSensor(Port.S2)
robot = DriveBase(left_motor, right_motor, wheel_diameter=33.5, axle_track=158)
robot.settings(MAX_SPEED, 2000, 300, 300)


# Configuración del PID Controlador
left_motor.control.pid(kp=10, ki=5, kd=3, integral_range=5, integral_rate=20, feed_forward=0);
right_motor.control.pid(kp=10, ki=5, kd=3, integral_range=5, integral_rate=20, feed_forward=0);

# Ajustar tolerancias (para que detecte movimientos mínimos)
left_motor.control.target_tolerances(speed=1, position=0.5);  # Solo 0.5° de margen de error
right_motor.control.target_tolerances(speed=1, position=0.5);


# Limitar velocidad y aceleración para evitar sobrepasos
left_motor.control.limits(speed=200, acceleration=200, actuation=100);
right_motor.control.limits(speed=200, acceleration=200, actuation=100);




# Configuración de Bluetooth
SERVER = 'gyrosens'
client = BluetoothMailboxClient()
mbox = TextMailbox('greeting', client)
gyro = NumericMailbox('gyro', client)
boleano = LogicMailbox('boleano', client)

# Conectar al servidor
print('Estableciendo conexión...')
client.connect(SERVER)
print('¡Conectado!')



# -----------------------------------------
# Variables globales y configuración
# -----------------------------------------
prox_dist = 25
trayectoria = []
pvc_color = "None"

# -----------------------------------------
# Enumeraciones y Clases
# -----------------------------------------
class Colors(Enum):
    RED = "Red"
    BROWN = "Brown"
    YELLOW = "Yellow"
    WHITE = "White"
    BLUE = "Blue"
    GREEN = "Green"
    BLACK = "Black" #uwu
    UNKNOW = "Unknow"

class Direction(Enum):
    NORTH = "North"
    EAST = "East"
    SOUTH = "South"
    WEST = "West"

class Node:
    def __init__(self, name):
        self.name = name
        self.left = None
        self.right = None
        self.up = None
        self.down = None
    
    def set_directions(self, left, right, up, down):
        self.left = left
        self.right = right
        self.up = up
        self.down = down
# Funciones auxiliares

# -----------------------------------------
# -----------------------------------------
def rgb_to_hsv(r, g, b):
    r, g, b = r / 100.0, g / 100.0, b / 100.0
    mx = max(r, g, b)
    mn = min(r, g, b)
    df = mx - mn
    h = 0 if mx == mn else (60 * ((g - b) / df) + 360) % 360 if mx == r else (60 * ((b - r) / df) + 120) % 360 if mx == g else (60 * ((r - g) / df) + 240) % 360
    s = 0 if mx == 0 else (df / mx) * 100
    v = mx * 100
    return h, s, v

def detect_color(sensor):
    a = sensor.rgb()
    if 30 > a[0] < a[2] and 30 > a[1] < a[2] and 27 < a[2] < 60:
        return Colors.BLUE;
    elif a[0] < 15 and a[1] < 20 and a[2] <= 20:
        return Colors.BLACK;
    elif a[0] > 30 and a[1] < 21:
        return Colors.RED;
    elif a[2] > 88:
        return Colors.WHITE;
    elif abs(a[0] - a[1]) <= 4 and 70 > a[1] >= 28 and a[2] < 40 and a[0] <= 70:
        return Colors.YELLOW;
    return Colors.UNKNOW;


def verify_obstacle()->bool: #Verifica si el objeto está a una distancia menor en el sensor.
    return infra_sensor.distance() < DETECT_DISTANT;
    

# -----------------------------------------
# Funciones de movimiento
# -----------------------------------------
def stop(): #Función para que se detenga el robot
    robot.stop()
    left_motor.brake()
    right_motor.brake()
    
    
def right_turn():
    stop()
    
    left_motor.run_target(400, left_motor.angle() + 200, Stop.HOLD, wait=False)
    right_motor.run_target(400, right_motor.angle() - 200, Stop.HOLD, wait=False)
    stop();

def left_turn():
    stop()
    left_motor.run_target(400, left_motor.angle() - 200, Stop.HOLD, wait=False)
    right_motor.run_target(400, right_motor.angle() + 200, Stop.HOLD, wait=False)    
    stop()
    
    
    
def move_distance(distance_mm, speed=200):
    wheel_diameter = 33.5
    rotation_degrees = (360 * distance_mm) / (m.pi * wheel_diameter)
    left_motor.run_target(speed, left_motor.angle() + rotation_degrees, Stop.HOLD, wait=False)
    right_motor.run_target(speed, right_motor.angle() + rotation_degrees, Stop.HOLD, wait=True)

def turn_degrees(angle, speed=100):
    axle_track = 158
    wheel_diameter = 33.5
    rotation_degrees = (axle_track * m.pi * angle) / (360 * wheel_diameter) * 360
    left_motor.run_angle(speed, -rotation_degrees, Stop.HOLD, wait=False)
    right_motor.run_angle(speed, rotation_degrees, Stop.HOLD, wait=True)


def move_left_motor(angle: int ): #Mueve el motor izquierdo un determinado angulo
    left_motor.run_angle(MAX_SPEED, angle, Stop.HOLD, wait=False );


def move_right_motor(angle:int ): # Mueve el motor derecho un determinado angulo.
    right_motor.run_angle(MAX_SPEED, angle, Stop.HOLD, wait=False);


 #Mueve el robot hacia adelante exactamente 1 "bloque lógico"
def move_one_block(block_size_mm=100):
    
    rotation_degrees = (360 * block_size_mm) / (m.pi * WHEEL_DIAMETER);  # Convert distance to motor degrees
    left_motor.run_target(MAX_SPEED, left_motor.angle() + rotation_degrees, Stop.HOLD, wait=False);
    right_motor.run_target(MAX_SPEED, right_motor.angle() + rotation_degrees, Stop.HOLD, wait=True);



# -----------------------------------------
# Funciones para alinear el robot
# -----------------------------------------

#Suponiendo la existencia de 3 sensores de color al frente.


def align_robot_start():
    
    
    while(True):
        
        left_color = detect_color(c_sensor1);
        center_color = detect_color(c_sensor2);
        right_color = detect_color(c_sensor3);
        
    # Existen tres posibles casos:
    # Tomando que: c_sensor1 es el de la izquierda, y c_sensor3 hasta la derecha.
    
    #1. Caso donde el c_sensor1 difiere de los otros dos
        if(center_color== right_color and left_color != center_color):
            #Se mueve ligeramente a la izquierda para alinearse
            move_left_motor(5);
        
    
    #2. Caso donde el c_sensor3 difiere de los otros dos
        elif(center_color == left_color and right_color != center_color):
            move_right_motor(5);
             
    pass 
    


# -----------------------------------------
# Funciones para llegar a la parte Azul en N posición
# -----------------------------------------

def arrive_first_time():
    
    #En primer caso, una vez el robot se ha alineado, basta con verificar que color se encuentr adelante

    
    pass 


#Funciones auxiliares

def move_if_find_red_first(): #Si se encuentra con una pared roja, los movimientos a realizar
     
    right_turn(); #Moverse 180 grados
    right_turn();
    move_one_block(); #Se mueve hacia adelante
    
    # Girar a la izquierda, y verificar que no hay nada que obstrulla.
    if (verify_obstacle()): #Si la distancia es menor a 25
        #Tratar de seguir adelante 
        move_one_block();
        move_one_block();
    
    #Moverse a la izquierda
    left_turn(); #Gira a la izquierda
    
    while(detect_color(c_sensor1) == Colors.WHITE and
          detect_color(c_sensor2) == Colors.WHITE and 
          detect_color(c_sensor3) == Colors.WHITE
          ): #Moverse hasta que encuentre la parte azul.
        
        move_one_block();
    
    
    if(detect_color(c_sensor1) == Colors.BLACK and
    detect_color(c_sensor2) == Colors.BLACK):
        move_red_to_black();
          

def move_if_find_black_first(): #Si lo primero que encontró es negro, entonces
    
    right_turn(); #GIra primero a la derecha
    
    move_distance(distance_mm=100); #Se mueve ligeramente para verificar
    
    #SI es blanco, entonces facilmente llega avanzando una casilla, recto y mientras sea blanco
    if(detect_color(c_sensor1) == Colors.WHITE and
       detect_color(c_sensor3) == Colors.WHITE):
        
        move_one_block();
        left_turn();
        
        while(detect_color(c_sensor1) == Colors.WHITE and
          detect_color(c_sensor2) == Colors.WHITE and 
          detect_color(c_sensor3) == Colors.WHITE
          ): #Moverse hasta que encuentre la parte azul.
        
            move_one_block();
    
        
    
    elif(detect_color(c_sensor1) == Colors.RED and
         detect_color(c_sensor3) == Colors.RED):
        move_if_find_red_first();
    
    pass;
    


def move_red_to_black(): #Significa que al girar a la izquierda encontró negro. ENtonces, basta con que gire 180% y recto
    
    right_turn();
    right_turn();
    
    while(detect_color(c_sensor1) == Colors.WHITE and
        detect_color(c_sensor2) == Colors.WHITE and
        detect_color(c_sensor3) == Colors.WHITE):        
        move_one_block();
    #Entonces llego a la parte azul.
# -----------------------------------------
# Main Loop
# -----------------------------------------
while True: #Serie de pasos a ejecutar
    
    
    align_robot_start(); #Primero, alinearse con el mapa
    arrive_first_time(); #Llegar por primera vez a la parte azul
    #Luego: Función para determinar su posición inicial y determinar el mapa
    #Función para colocarse en una esquina IZQ o DER
    
    #Generar BUCLE QUE; CONOCIENDO POSICIONES GENERA LAS RUTAS DE BUSQUEDA Y COLECTA.
    
    pass
