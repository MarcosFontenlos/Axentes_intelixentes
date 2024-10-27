# import rospy
import tf
import numpy as np

# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan
# from odometry.msg import Odometry
from typing import Union, List, Dict


def tolerancia_cartesiana(
    ideal: tuple[float, float], real: tuple[float, float], tolerancia: float
) -> bool:
    """Comprueba si la posición real del robot está dentro de la tolerancia cartesiana.
    - Recibe la posición ideal y la posición real del robot y la tolerancia cartesiana.
    - Devuelve True si la posición real del robot está dentro de la tolerancia cartesiana y False en caso contrario.
    """
    if (ideal[0] - tolerancia) <= real[0] <= (ideal[0] + tolerancia) and (
        ideal[1] - tolerancia
    ) <= real[1] <= (ideal[1] + tolerancia):
        return True
    else:
        return False


def obtener_punto_cardinal(yaw: float, tolerancia: float = 0.1) -> str:
    """Devuelve la dirección cardinal según el valor de yaw (orientación del robot)."""
    yaw = yaw % (2 * np.pi)  # Normalizamos entre 0 y 2π

    if abs(yaw - 0) < tolerancia or abs(yaw - 2 * np.pi) < tolerancia:
        return "Norte"
    elif abs(yaw - np.pi / 2) < tolerancia:
        return "Este"
    elif abs(yaw - np.pi) < tolerancia:
        return "Sur"
    elif abs(yaw - 3 * np.pi / 2) < tolerancia:
        return "Oeste"
    else:
        return "Orientación indefinida"


class Estado:
    INICIO = 1  # Estado inicial del robot
    EXPLORACION = 2  # Estado para explorar el laberinto, se mueve por los caminos (que se presuponen rectos) entre nodos o entre nodo y final sin camino. La orientación inical del robot se determina en los estados de inicio, análisis y esquiva obstáculo.
    ANALISIS = 3  # Estado para analizar el nodo, se detiene en el nodo y gira para explorar los caminos.
    RETROCESO = 4  # Estado para retroceder en el laberinto, se ejecuta cuando el robot se encuentra en un callejón sin salida.
    ESQUIVAR_OBSTACULO = 5  # Estado para esquivar obstáculos, se detiene en el nodo y gira para esquivar obstáculos.
    FINALIZADO = 6  # Estado para el final del laberinto
    EXCEPCION = 7  # Estado para excepciones


class Nodo:
    def __init__(self, posicion: tuple[float, float]):
        # Definimos la estructura de nodo con tipos específicos
        self.nodo: Dict[str, Union[tuple[float, float], List[str]]] = {
            "posicion": posicion,  # La posición es una tupla de float
            "caminos": ["None", "None", "None", "None"],  # [norte, este, sur, oeste]
        }

    def actualizar_direcciones(self, direcciones: List[str]):
        """Actualiza las direcciones de los caminos del nodo despues de explorarlo con el sensor láser.
        -  Recibe una lista de 4 direcciones ordenadas: norte, este, sur, oeste, que en cada posición puede ser "None" o "Blanco". (Se trata de la inicialización de los caminos del nodo).
        -  Modifica el atributo 'caminos' del nodo, cambiando "None por blanco si hay camino libre en primera exploración.
        """
        if len(direcciones) == 4 and all(
            direccion in ["None", "Blanco"] for direccion in direcciones
        ):
            self.nodo["caminos"] = direcciones
        else:
            raise ValueError(
                "Se esperan exactamente 4 direcciones del reconocimiento del nodo."
            )

    def marcar_camino(self, direccion: str):
        """Marca el camino en la dirección especificada.
        - Recibe una orientación (norte, este, sur, oeste) y modifica el atributo 'caminos' del nodo, cambiando "Blanco" por "Gris" si se ha pasado por el camino exactamente una vez y "Gris" por "Negro" si se ha pasado dos veces.
        - Modifica el atributo 'caminos' del nodo cambiando el estado del camino del que se viene y del camino hacia el que se dirige.
        """

        direcciones = ["Norte", "Este", "Sur", "Oeste"]
        if direccion in direcciones:
            idx = direcciones.index(direccion)  # Encuentra el índice de la dirección
            print(idx)
            # Lógica para marcar el camino
            if (
                self.nodo["caminos"][idx] == "None"
                or self.nodo["caminos"][idx] == "Blanco"
            ):
                self.nodo["caminos"][idx] = "Blanco"  # type: ignore
            elif self.nodo["caminos"][idx] == "Gris":
                self.nodo["caminos"][idx] = "Negro"  # type: ignore
            else:
                raise ValueError(
                    "El camino se ha recorrido más de dos veces: el algoritmo no ha funcionado"
                )

    def obtener_estado_caminos(self) -> List[str]:
        """Devuelve el estado de los caminos.
        Retorna: una lista con los valores de estado de las orientaciones del nodo"""
        return self.nodo["caminos"]  # type: ignore

    def obtener_posicion(self) -> tuple[float, float]:
        """Devuelve la posición del nodo."""
        return self.nodo["posicion"]  # type: ignore

    def obtener_estado_camino(self, punto_cardinal: str) -> str:
        direcciones = ["Norte", "Este", "Sur", "Oeste"]
        if punto_cardinal in direcciones:
            idx = direcciones.index(punto_cardinal)
            return str(self.nodo["caminos"][idx])
        else:
            raise ValueError(
                f"Dirección '{punto_cardinal}' no válida. Debe ser una de {direcciones}."
            )

    def obtener_siguiente_camino(self) -> Union[str, None]:
        """Devuelve el punto cardinal del siguiente camino a seguir según las reglas de Tremaux.
        Retorna None si no hay caminos disponibles.
        """
        direcciones = ["Norte", "Este", "Sur", "Oeste"]  # Mapeo de direcciones
        lista_caminos = self.nodo["caminos"]

        # Buscar el primer camino blanco
        for idx, camino in enumerate(lista_caminos):
            if camino == "Blanco":
                return direcciones[idx]  # Retorna la dirección correspondiente

        # Si no hay caminos blancos, buscar el primer camino gris
        for idx, camino in enumerate(lista_caminos):
            if camino == "Gris":
                return direcciones[idx]  # Retorna la dirección correspondiente

        return None  # Si no hay caminos disponibles


class LaberintoTremaux:
    def __init__(self) -> None:
        self.estado = Estado.INICIO
        self.estado_anterior = None
        self.lista_nodos: List[Nodo] = []

        # Suscripción a /odom y variables para almacenar pose y orientación
        self.odom_sub = rospy.Subscriber("/robot/odom", Odometry, self.odom_callback)
        self.pose: tuple[float, float] = (0.0, 0.0)  # Inicializamos la posición (x, y)
        self.yaw: float = 0.0  # Inicializamos la orientación (yaw) en 2D

        # Tolerancias
        self.tolerancia_cartesiana: float = 0.05  # Margen de error en la posición
        self.tolerancia_angular: float = 0.15  # Margen de error en la orientación

    def odom_callback(self, msg):
        # Actualizar la posición (x, y)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.pose = (
            self.x,
            self.y,
        )  # Actualizar la tupla pose con los valores de posición

        # Extraer cuaternión para la orientación
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]

        # Convertir el cuaternión a ángulo yaw (solo giro sobre el eje Z)
        (_, _, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        # Actualizar el ángulo de giro (yaw)
        self.yaw = yaw

    def deteccion_nodo(self):  # METODO ANALISIS
        """Método de detección de nodo en el laberinto. Detecta si el robot se encuentra en un nodo. Funciona como trasicion entre estados. Reconoce patrones semejantes a intersecciones mediante la deteccion del sensor laser."""

    def agregar_nodo(self, posicion: tuple[float, float]):  # METODO ANALISIS
        """Agrega un nodo a la lista de nodos."""
        nuevo_nodo = Nodo(posicion)
        self.lista_nodos.append(nuevo_nodo)

    def obtener_ultimo_nodo(self) -> Nodo:
        """Devuelve el último nodo de la lista."""
        return self.lista_nodos[-1]

    def mapeo_nodo(self, nodo: Nodo):  # METODO ANALISIS
        """Método de mapeo de nodo en el laberinto. Mapea el nodo en el laberinto para determinar las direcciones de los caminos disponibles. Consiste en girar 360º para detectar la orientación de las vías libres"""

    def orientar_robot(self, punto_cardinal: str):  # METODO ANALISIS
        """Orienta el robot hacia el punto cardinal especificado: Girar el robot hacia la dirección especificada."""
        pass

    def inicio(self):
        """Método de estado inicio."""
        pass

    def explorar(self):
        """Método de estado exploración."""
        pass

    def analisis(self):
        """Método de estado análisis.
        1º Comprueba si el nodo ya ha sido visitado.
        2º Actualiza las direcciones de los caminos del nodo: si es la primera vez que se visita el nodo se actualizan las direcciones de los caminos del nodo; si ya se ha visitado se limita a marcar el camino tanto de procedencia como el que se sigue.
        3º Decide la dirección a seguir siguiendo las reglas de Tremaux:
            - No siga el mismo camino dos veces: si al llegar y actualizar el estado del camino de procedencia, ese camino se marca "Negro", ya no se podrá volver a tomar.
            - Si llega a un cruce nuevo, no importa qué camino siga: se selecciona el primer camino situado a la derecha del de origen, que ha de estar marcado de "Blanco" necesariamente.
            - Si un camino nuevo lo lleva a un cruce viejo, o a un callejón sin salida, retroceda hasta la entrada del camino: que un camino nuevo lo lleve a un cruze viejo implica que, una vez marcado el camino, la orientación de procedencia ha de ser "Gris"; un callejón sin salida se caracteeriza de manera diferente (fuerza el giro de 180º y retorno hacia el nodo precedente)
            - Si un camino viejo lo lleva a un cruce viejo, tome un camino nuevo, y si no lo hay, tome cualquiera: un camino viejo en un cruze viejo implica que el camino de procedencia esté marcado en "Negro" después de la actualización.
        """
        orientacion_precedente = (self.yaw - np.pi) % (
            2 * np.pi
        )  # La orientacion precedente es la diametralmente opuesta a la que tiene el robot, normalizada entre 0 y 2pi
        punto_cardinal_precedente = obtener_punto_cardinal(
            orientacion_precedente, self.tolerancia_angular
        )
        nodo_visitado = False
        for nodo in self.lista_nodos:
            if tolerancia_cartesiana(
                nodo.obtener_posicion(), self.pose, self.toleracia_cartesiana
            ):  # En este caso el nodo ya ha sido visitado: se desarrolla la logica de actualización de camino de procedencia, decisión, giro, y actualización camino elegido. SI EL CAMINO DE PRECEDENCIA ES GRIS: RETROCESO; SI EL CAMINO DE PRECEDENCIA ES NEGRO, PRIMER CAMINO BLANCO DESDE LA DERECHA DE LA ENTRADA Y SI NO EXISTE NINGUNO BLANCO, PRIMER CAMINO GRIS DESDE LA DERECHA IGUALMENTE. SI TODOS LOS CAMINOS SON NEGROS: EXCEPCION.
                nodo_visitado = True
                nodo.marcar_camino(punto_cardinal_precedente)
                if nodo.obtener_estado_camino(punto_cardinal_precedente) == "Gris":
                    nodo.marcar_camino(punto_cardinal_precedente)
                    self.estado = Estado.RETROCESO
                elif nodo.obtener_estado_camino(punto_cardinal_precedente) == "Negro":
                    camino_siguiente = nodo.obtener_siguiente_camino()
                    if camino_siguiente == None:
                        self.estado = Estado.EXCEPCION
                    else:
                        self.orientar_robot(camino_siguiente)
                        nodo.marcar_camino(camino_siguiente)
                        self.estado = Estado.EXPLORACION
                else:
                    raise Exception("blablabla")
        if not nodo_visitado:
            self.agregar_nodo(
                posicion=self.pose
            )  # En este caso el nodo no se ha visitado con anterioridad, por lo que la logica consiste en situarse en el camino libre a la derecha del camino de procedencia. Se debe marcar este como "Gris" en primera instancia, y una vez situado, el camino de destino tambien.
            nodo = self.obtener_ultimo_nodo()
            nodo.marcar_camino(punto_cardinal_precedente)
            camino_siguiente = nodo.obtener_siguiente_camino()
            if camino_siguiente == None:
                self.estado = Estado.EXCEPCION
            else:
                self.orientar_robot(camino_siguiente)
                nodo.marcar_camino(camino_siguiente)
                self.estado = Estado.EXPLORACION

    def retroceso(self):
        pass

    def esquivar_obstaculos(self):
        pass

    def detener(self):
        pass

    def ejecucion(self):
        # Manejo de estados
        if self.estado == Estado.INICIO:
            self.inicio()
        elif self.estado == Estado.EXPLORACION:
            self.explorar()
        elif self.estado == Estado.ANALISIS:
            self.analisis()
        elif self.estado == Estado.ESQUIVAR_OBSTACULO:
            self.esquivar_obstaculos()
        elif self.estado == Estado.RETROCESO:
            self.retroceso()
        elif self.estado == Estado.FINALIZADO:
            self.detener()
        elif self.estado == Estado.EXCEPCION:
            self.detener()
        else:
            self.detener()


nodo = Nodo((0, 0))
nodo.marcar_camino("este")
print(nodo.obtener_estado_caminos())
