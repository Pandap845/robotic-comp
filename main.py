#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.nxtdevices import LightSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math as m
from pybricks.messaging import BluetoothMailboxServer, TextMailbox, NumericMailbox, LogicMailbox, BluetoothMailboxClient
from enum import Enum

"""
-----------------------------------------
Inicializacion de los sensores y motores
-----------------------------------------
"""
ev3 = EV3Brick()
left_motor = Motor(Port.D)  
right_motor = Motor(Port.A)
serv = Motor(Port.C)
claw = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
c_sensor2 = ColorSensor(Port.S3)
c_sensor1 = ColorSensor(Port.S4)
prox = InfraredSensor(Port.S2)
gyroscope = GyroSensor(Port.S1)
robot = DriveBase(left_motor, right_motor, wheel_diameter=33.5, axle_track=158)
robot.settings(1000, 2000, 300, 300 )
SERVER = 'gyrosens'
client = BluetoothMailboxClient()
mbox = TextMailbox('greeting', client)
gyro = NumericMailbox('gyro', client)
boleano = LogicMailbox('boleano', client)

"""
SETUP
"""
garra = False
obstaculo = False
atrapado = False
trayectoria = []
prox_dist = 25 #Proximidad a la cual el robot persibe un obstaculo
max_dist = (120)*60 #Angulo de rotacion maximo para que el motor no pase de 90 grados al alinerase
colores = ["BLANCO", "ROJO"]
colores1 = ["BLANCO", "ROJO"]
colores2 = ["BLANCO", "AMARILLO"]
gyro_angle = 0
target = 15
gain = 1.2
dist = False
Align = False
Sieze = False
ready = False
entrega = False
pvc_color = "None"

#Connection to SERVER
print('establishing connection...')
client.connect(SERVER)
print('connected!')

#Mapa

#Creacion de la matriz que representa el mapa
nodos = [[1, 1, 1, 1, 1, 1, 1],
         [1, 0, 1, 0, 1, 0, 1],
         [1, 0, 0, 0, 0, 0, 1],
         [1, 0, 1, 0, 1, 0, 1],
         [1, 0, 0, 0, 0, 0, 1],
         [1, 0, 1, 0, 1, 0, 1],
         [1, 1, 1, 1, 1, 1, 1]]

#Asignación de letras a los nodos que son puertas
nodos[1][1] = "A"
nodos[1][3] = "F"
nodos[2][2] = "D"
nodos[2][4] = "I"
nodos[3][1] = "B"
nodos[3][3] = "G"
nodos[4][2] = "E"
nodos[4][4] = "J"
nodos[5][1] = "C"
nodos[5][3] = "H"
nodos[5][5] = "K"

#Puertas
museum = ["E", "H"]
library = ["J", "K"]
city_hall = ["J", "I"]
drug_store = ["B", "G"]
bakery = ["A", "D"]
school = ["I", "F"]
park = ["A", "B", "C"]



"""
Clase de los nodos
"""
# Enumeración de los colores
class Colors(Enum):
    RED = "Red"
    BROWN = "Brown"
    YELLOW = "Yellow"
    WHITE = "White"
    BLUE = "Blue"
    GREEN = "Green"

# Enum para las orientaciones del robot
class Direction(Enum):
    NORTH = "North"
    EAST = "East"
    SOUTH = "South"
    WEST = "West"
    
# Función para ajustar los colores basados en la orientación del robot
def adjust_color_direction(direction: Direction, front: Colors, right: Colors, back: Colors, left: Colors) -> tuple:
    if direction == Direction.NORTH:
        return (front, right, back, left)
    elif direction == Direction.EAST:
        return (left, front, right, back)
    elif direction == Direction.SOUTH:
        return (back, left, front, right)
    elif direction == Direction.WEST:
        return (right, back, left, front)

# Clase del Node
class Node:
    def __init__(self, val: int):
        self.val = val  #Valor que almacena el nodo
        self.left: Colors = None  # 
        self.right: Colors = None  # 
        self.up: Colors = None  # 
        self.down: Colors = None  # 
        
    def set_directions(self, left: Colors, right: Colors, up: Colors, down: Colors):
        # Determinar los colores a sus lados
        self.left = left
        self.right = right
        self.up = up
        self.down = down
    


# Nodos predeterminados. Suponiendo que el frente es el lado Azul
nodo_a = Node("Nodo A")
nodo_a.set_directions(Colors.RED, Colors.WHITE, Colors.YELLOW, Colors.YELLOW)

nodo_b = Node("Nodo B")
nodo_b.set_directions(Colors.WHITE, Colors.WHITE, Colors.YELLOW, Colors.YELLOW)

nodo_c = Node("Nodo C")
nodo_c.set_directions(Colors.WHITE, Colors.RED, Colors.BLACK, Colors.YELLOW)

nodo_d = Node("Nodo D")
nodo_d.set_directions(Colors.WHITE, Colors.BLACK, Colors.WHITE, Colors.WHITE)

nodo_e = Node("Nodo E")
nodo_e.set_directions(Colors.BLACK, Colors.YELLOW, Colors.WHITE, Colors.WHITE)

nodo_f = Node("Nodo F")
nodo_f.set_directions(Colors.RED, Colors.WHITE,  Colors.WHITE, Colors.BLACK)

nodo_g = Node("Nodo G")
nodo_g.set_directions(Colors.WHITE, Colors.WHITE, Colors.BLACK, Colors.YELLOW)

nodo_h = Node("Nodo H")
nodo_h.set_directions(Colors.WHITE, Colors.RED, Colors.BLACK, Colors.YELLOW)

nodo_i = Node("Nodo I")
nodo_i.set_directions(Colors.YELLOW, Colors.YELLOW, Colors.WHITE, Colors.WHITE)

nodo_j = Node("Nodo J")
nodo_j.set_directions(Colors.YELLOW, Colors.YELLOW, Colors.WHITE, Colors.WHITE)




#----------------------------------------------------------------------------------------------------------#
    """
    -------------------------------------
    -Funciones principales de movimiento-
    -------------------------------------
    """


def align(): # Función que debe de alinear el robot
    robot.drive(1000,-1); #Se va para atrás para comprobar
    if color(c_color1.rgb()) != color(c_color2.rgb()):    # SI se percata en ese momento que son dos colores distintos
        #Va a tratar a alinear conforme el primer color detectado. 
        while True:
                
            robot.stop();
            robot.drive(1000, -30); #Va hacia atrás en un angulo determinado
            if(color(c_color1.rgb()) == color(c_color2.rgb())) #En el momento en que ambos sean del mimso color.
                break;
    else:
        #EN caso en que ambos colores sigan siendo lo mismo, significa que está alineado con X color.
        

def stop(): #Función que detiene por completo al robot
    robot.stop()
    left_motor.brake()
    right_motor.brake()


def right(): #Función que hace que el robot gire a la derecha
    stop()
    right_motor.run(-400)
    left_motor.run(400)
    while gyroscope.angle() > -84:
        pass
    stop()

def left(): #FUnción que hace que el robot igre a la izquierda
    stop()
    left_motor.run(-400)
    right_motor.run(400)
    while gyroscope.angle() < 84:
        pass
    stop()
    
    
    
# Funciones de movimiento

def move_red_first(): #Función que ocurre al momento de encontrar la pared roja
    
    #EL robot debe de dar una vuelta de 180 grados.
    
    

def move_black_first(): #Función que ocurre al encontrar una pared negra 



def move_white_first(): #Función que ocurre al encontrar piso blanco.


#-------------------------------------------------------------------------------------------------------------
"""
-----------------------------------
AUXILIARES
-----------------------------------
""" 
    
# Función que convierte el color RGB detectado por el sensor en un HSV
def rgb_to_hsv(r, g, b):
    r, g, b = r / 100.0, g / 100.0, b / 100.0 #Normalización de valores
    mx = max(r, g, b)
    mn = min(r, g, b)
    df = mx-mn
    if mx == mn:
        h = 0
    elif mx == r:
        h = (60 * ((g-b)/df) + 360) % 360
    elif mx == g:
        h = (60 * ((b-r)/df) + 120) % 360
    elif mx == b:
        h = (60 * ((r-g)/df) + 240) % 360
    if mx == 0:
        s = 0
    else:
        s = (df/mx)#*100
    v = mx
    return h, (s * 100), (v * 100)


def color(a):

    b = "BLANCO"
    if 30 > a[0] < a[2] and 30 > a[1] < a[2] and 27 < a[2] < 60:
        b = "AZUL"
        print("AZUL")
    elif a[0] < 15 and a[1] < 20 and a[2] <= 20:
        b = "NEGRO"
        print("NEGRO")
    elif a[0] > 30 and a[1] < 21:
        b = "ROJO"
        print("ROJO")
    elif a[2] > 88:
        b = "BLANCO"
        print("BLANCO")
    elif abs(a[0]-a[1]) <= 4 and 70 > a[1] >= 28 and a[2] < 40 and a[0] <= 70:
        b = "AMARILLO"
        print("AMARILLO")
    return b


def color2(a):

    b = "BLANCO"

    print(a)

    if a[0] < 5 and a[1] < 15  and 18 < a[2]:
        b = "AZUL"
        print("AZUL")
    elif a[0] < 7 and a[1] < 5 and a[2] <= 5:
        b = "CAFE"
        print("CAFE")
    elif 15 < a[0] < 30 and a[1] < 5 and a[2] < 5:
        b = "ROJO"
        print("ROJO")
    elif a[0] < 15 and a[1] > 10 and a[2] < 30:
        b = "VERDE"
        print("VERDE")

    return b

# ------------------------------------------------------------------------------------------------
    """
    ------------------------
    MAIN LOOP
    -------------------------
    """
#Main Loop
while True:

   