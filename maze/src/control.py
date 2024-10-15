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
        lvector = len(msg.ranges) // 2
        lvector_right = len(msg.ranges) * 3 // 4
        lvector_left = len(msg.ranges) // 4
        self.range_ahead = min(msg.ranges[lvector-50:lvector+50])
        self.range_right = min(msg.ranges[lvector_left-25:lvector_left+25])
        self.range_left = min(msg.ranges[lvector_right-25:lvector_right+25])

    def avanzar(self):
        self.move_cmd.linear.x = 0.3
        self.move_cmd.angular.z = 0.0
        self.pub.publish(self.move_cmd)

    def girar_izquierda(self):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.5
        self.pub.publish(self.move_cmd)

    def girar_derecha(self):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = -0.5
        self.pub.publish(self.move_cmd)

    def detener(self):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.pub.publish(self.move_cmd)

    def sigue_pared(self):
        # Detectar el final del laberinto: espacio libre al frente y sin paredes a los lados
        if self.range_ahead > 6 and self.range_right > 6 and self.range_left > 6:
            self.estado = Estado.FINALIZADO
        elif self.range_ahead > 1 and self.range_right < 0.5:
            self.estado = Estado.AVANZA  # Avanza si no hay obstaculo en frente y hay una pared cerca
        elif self.range_ahead <= 1:
            self.estado = Estado.GIRA_IZQUIERDA  # Gira a la izquierda si hay un obstaculo enfrente
        elif self.range_right > 0.5:
            self.estado = Estado.GIRA_DERECHA  # Gira a la derecha si no hay pared cercana

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
        elif self.estado == Estado.SIGUE_PARED:
            self.sigue_pared()
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
