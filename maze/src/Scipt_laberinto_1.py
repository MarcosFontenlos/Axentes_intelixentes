#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math


def laser_callback(data):
 
 global ranges,range_mid,range_left,range_rigth,start,parar,adelante,final

 ranges=[5 if x == float('inf') or x == float('-inf') else x for x in data.ranges]
 range_rigth=ranges[5:50]
 range_mid=ranges[60:120]
 range_left=ranges[130:-5]

 if calcular_media(ranges)>4.75:
     final=True

 if max(range_rigth)<5:
     start=False


 if min(range_rigth)<3:
     adelante=False
 else:
     adelante=True


 if min(range_left)<2 or min(range_rigth)<2 or min(range_mid)<2:
     parar=True
     adelante=False
 else:
     parar=False
     adelante=True

  



def odom_callback(data):
    
    global initial_position,distance_total,previous_position,distance_total
    # Extraer la posicion actual del robot
    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y

    # Guardar la posicion inicial en la primera iteracio
    if initial_position is None:
        initial_position = (current_x, current_y)
        previous_position= (current_x, current_y)

    # Calcular la distancia desde la posicion inicial
    distance = math.sqrt((current_x - initial_position[0])**2 + (current_y - initial_position[1])**2)
      # Calcular la distancia entre la posicion anterior y la actual
    distance_step = math.sqrt((current_x - previous_position[0])**2 + (current_y - previous_position[1])**2)

    # Acumular la distancia recorrida
    distance_total += distance_step
    # Actualizar la posicion anterior
    previous_position = (current_x, current_y)




def calcular_media(lista):
    return sum(lista) / (len(lista))


def orientar_choque():
    global range_mid , range_rigth,range_left,pub,velocity,parar,ranges

    min_value=min((min(range_mid),min(range_rigth),min(range_left)))
    min_index=ranges.index(min_value)
    valor_maximo=max(ranges)
    indice_maximo = ranges.index(valor_maximo)


    if calcular_media(range_left) > max( calcular_media(range_rigth), calcular_media(range_mid)) and min(range_left)>(min(min(range_rigth),min(range_mid))):
        rospy.loginfo("Vamos a girar a la izquierda (orientacion_choque)")
        rospy.loginfo("Mean range left: %.2f, Mean range right: %.2f, Mean range mid: %.2f" % 
              (calcular_media(range_left), calcular_media(range_rigth), calcular_media(range_mid)))

        velocity.linear.x=0.15
        velocity.angular.z=0.65
        pub.publish(velocity)
    elif calcular_media(range_rigth) >max(calcular_media(range_left),calcular_media(range_mid) ) and min(range_rigth)>(min(min(range_left),min(range_mid))):
        rospy.loginfo("Vamos a girar a la derecha   (orientacion_choque)")
        rospy.loginfo("Mean range left: %.2f, Mean range right: %.2f, Mean range mid: %.2f" % 
              (calcular_media(range_left), calcular_media(range_rigth), calcular_media(range_mid)))

        velocity.linear.x=0.15
        velocity.angular.z=-0.65
        pub.publish(velocity)
    else:
        velocity.linear.x=0.5
        velocity.angular.z=0
        pub.publish(velocity)
        rospy.loginfo("Vamos de frente  (orientacion_choque)")
        rospy.loginfo("Mean range left: %.2f, Mean range right: %.2f, Mean range mid: %.2f" % 
              (calcular_media(range_left), calcular_media(range_rigth), calcular_media(range_mid)))
    rospy.loginfo("MIN range left: %.2f, MIN range right: %.2f, MIN range mid: %.2f" % 
              (min(range_left), min(range_rigth), min(range_mid)))


def orientar():
    global range_mid , range_rigth,range_left,pub,velocity
    if (calcular_media(range_mid) >calcular_media(range_rigth)) and  (calcular_media(range_mid) >calcular_media(range_left)):
        rospy.loginfo("Vamos a de frente(orientacion)")
        velocity.linear.x=1
        velocity.angular.z=0
        pub.publish(velocity)
    elif (calcular_media(range_left) > calcular_media(range_rigth)) :      
        rospy.loginfo("Vamos a girar a la izquierda (orientacion)")
        velocity.linear.x=0
        velocity.angular.z=0.5
        pub.publish(velocity)
    elif (calcular_media(range_rigth) > calcular_media(range_left)) :
        rospy.loginfo("Vamos a girar a la derecha (orientacion)")
        velocity.linear.x=0
        velocity.angular.z=-0.5
        pub.publish(velocity)
    else:
        rospy.loginfo("No se esta cumpliendo bien la logica")

ranges=[0 , 0]
range_mid=[0,0]
range_left=[0,0]
range_rigth=[0,0 ]
velocity=Twist()

initial_position = None  # Inicialmente no tenemos la posicin inicial
distance_total= 0 #Distancia total recorrida por el robot
previous_position=None

start_time = time.time()
rospy.init_node('robot_control_node')
rospy.loginfo("Inicio de la busqueda de salida.")
sub_odom= rospy.Subscriber('/robot/odom', Odometry,odom_callback)
# Suscriptor al topico de escaneo laser
sub=rospy.Subscriber('/robot/laser/scan', LaserScan,laser_callback)
# Publicador al topico de velocidad del robot
pub=rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)

# Suscriptor al tooico de odometria
#sub_odom= rospy.Subscriber('/robot/odom', Odometry,odom_callback)
# Variable para controlar la velocidad
#rospy.spin()
# Bucle principal (opcional)
rate = rospy.Rate(5)  # 10 Hz

start=True
parar=False
adelante=False
final=False

while not  rospy.is_shutdown():
    if final:
        end_time = time.time()
        total_time = end_time - start_time
        rospy.loginfo("El robot ha encontrado la salida.")
        rospy.loginfo("Tiempo total de busqueda:%.2f segundos.",(total_time))
        rospy.loginfo("Distancia total de bsqueda:%.2f m.",(distance_total))
        velocity.linear.x=0
        velocity.angular.z=0
        pub.publish(velocity)
            # Parar el nodo de ROS y el programa
        rospy.signal_shutdown("El robot ha encontrado la salida. Programa finalizado.")
    elif parar: 
        velocity.linear.x=0
        velocity.angular.z=0
        pub.publish(velocity)
        orientar_choque()
    
    elif start: 
        velocity.linear.x=0.5
        velocity.angular.z=0
        pub.publish(velocity)
        
    elif adelante:
        velocity.linear.x=0.5
        velocity.angular.z=0
        pub.publish(velocity)

    else:
        orientar()

    rate.sleep()  # Esperar hasta el siguiente ciclo del bucle


