#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Definicion de los estados de la maquina
class Estado:
    SIGUE_PARED = 1
    GIRA_IZQUIERDA = 2
    GIRA_DERECHA = 3
    AVANZA = 4
    DETENIDO = 5
    FINALIZADO = 6  # Estado para el final del laberinto
    GIRA_IZQUIERDA_AVANZO = 7
    GIRA_DERECHA_AVANZO = 8
    BUSCA_PARED = 9
class RobotLaberinto:
    def __init__(self):
        self.estado = Estado.DETENIDO  # Estado inicial del robot
        self.estado_anterior = None  # Para detectar cambios de estado
        self.range_ahead = 2          # Distancia al obstaculo al frente
        self.range_right = 2          # Distancia a la pared a la derecha
        self.range_left = 2           # Distancia a la pared a la izquierda
        self.pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)  # Publicador de movimientos
        self.move_cmd = Twist()        # Comandos de movimiento
        rospy.Subscriber('/robot/laser/scan', LaserScan, self.scan_callback)  # Suscriptor de laser

    def scan_callback(self, msg):
        # Actualizar las distancias segun la informacion del sensor laser
        
        self.range_ahead = min(msg.ranges[70:110])
        self.range_right = min(msg.ranges[160:-1])
        self.range_left = min(msg.ranges[0:30])

    def avanzar(self):
        self.move_cmd.linear.x = 0.3
        self.move_cmd.angular.z = 0.0
        self.pub.publish(self.move_cmd)

    def girar_izquierda(self):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.5
        self.pub.publish(self.move_cmd)

    def girar_izquierda_avanzo(self):
        self.move_cmd.linear.x = 0.2
        self.move_cmd.angular.z = 0.5
        self.pub.publish(self.move_cmd)

    def girar_derecha(self):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = -0.5
        self.pub.publish(self.move_cmd)

    def girar_derecha_avanzo(self):
            self.move_cmd.linear.x = 0.2
            self.move_cmd.angular.z = -0.5
            self.pub.publish(self.move_cmd)

    def detener(self):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.pub.publish(self.move_cmd)

    def sigue_pared(self):
        if self.range_ahead > 1:  # Si hay espacio por delante, avanza
            self.move_cmd.linear.x = 0.3
            if self.range_right < 0.5:  # Si hay una pared muy cerca a la derecha, gira a la izquierda
                self.move_cmd.angular.z = 0.25
            elif self.range_right > 0.9:  # Si la pared esta lejos a la derecha, gira a la derecha
                self.move_cmd.angular.z = -0.5
            else:  # Avanza recto si la pared esta a una distancia correcta
                self.move_cmd.angular.z = 0.0
        else:  # Si no hay espacio por delante, gira a la izquierda
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.5
        self.pub.publish(self.move_cmd)

    def buscar_pared(self):
        # Si no ha encontrado una pared, sigue avanzando hasta detectar una
        self.move_cmd.linear.x = 0.3
        self.move_cmd.angular.z = 0.0
        if self.range_ahead < 0.8:
            self.hay_pared = True
            self.estado = Estado.SIGUE_PARED
        self.pub.publish(self.move_cmd)

    def ejecutar(self):
        # Comprobar si el estado ha cambiado
        if self.estado != self.estado_anterior:
            print("Cambio de estado: {0} -> {1}".format(self.estado_anterior, self.estado))
            self.estado_anterior = self.estado
	
        # Imprimir el estado actual
            print("Estado actual: ", self.estado)
            print(self.range_ahead)
            print(self.range_left)
            print(self.range_right)
        # Ejecutar el comportamiento segun el estado actual
        if self.estado == Estado.AVANZA:
            self.avanzar()
        elif self.estado == Estado.GIRA_IZQUIERDA:
            self.girar_izquierda()
        elif self.estado == Estado.GIRA_DERECHA:
            self.girar_derecha()
        elif self.estado == Estado.GIRA_IZQUIERDA_AVANZO:
            self.girar_izquierda_avanzo()
        elif self.estado == Estado.GIRA_DERECHA_AVANZO:
            self.girar_derecha_avanzo()
        elif self.estado == Estado.SIGUE_PARED:
            self.sigue_pared()
        elif self.estado == Estado.BUSCA_PARED:
            self.buscar_pared()

        elif self.estado == Estado.FINALIZADO:
            print("Labertinto resuelto: el robot ha encontrado la salida")
            self.detener()
            rospy.signal_shutdown("Final del laberinto")
        else:
            self.detener()

    def correr(self):
        rate = rospy.Rate(4)
        self.estado = Estado.SIGUE_PARED  # Estado inicial
        self.estado_anterior = None  # Para detectar cambios de estado
        while not rospy.is_shutdown():
            self.ejecutar()  # Ejecutar la accion correspondiente
            rate.sleep()

def main():
    rospy.init_node('movedor_robot')
    robot = RobotLaberinto()
    robot.correr()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Nodo terminado.")