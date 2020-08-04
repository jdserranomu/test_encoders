#!/usr/bin/env python
# Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy, math, roslaunch, time
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
import networkx as nx
from test_encoders.srv import StartService
from test_encoders.srv import TerminarRecorrido
from std_msgs.msg import Int32
import matplotlib.pyplot as plt


# Clase que representa una casilla, tiene ubicacion o punto que la define (mitad) y si un objeto la cubre o no.
class Casilla:
    def __init__(self, xP, yP, pE):
        self.x = float(xP)
        self.y = float(yP)
        self.libre = pE


# Clase que hace referencia a una posicion en especifico.
class Posicion:
    def __init__(self, xP, yP, pTeta):
        self.x=float(xP)
        self.y=float(yP)
        self.teta=pTeta


# Iniciar grafico de networkx
g = nx.Graph()
# Distancia entre centro de cuadriculas, 10 debe ser divisible por esta distancia
distanciaCuadricula = 40  #5  # .8 #1 ############# CAMBIO ###########
totalx = 2000                         ############# CAMBIO ###########
# Numero de cuadriculas de la escena seleccionada eje x
n = int(totalx/distanciaCuadricula)
# Numero de cuadriculas de la escena seleccionada eje y
totaly = 1000  			     ############# CAMBIO ###########
m = int(totaly/distanciaCuadricula)
# Arreglo con la informacion de cada una de las casillas
casillas = []
# Posicion actual del robot, debe actualizarse por el topico
posicionActual = Pose()
# Posicion final del robot, inicialmente se toma como la cuadricula superior derecha con angulo 0
posicionFinal = Pose()
# Es el diametro de la rueda en milimetros
diametroRueda = 29.3  # milimetros
# Es el radio de la rueda del Pioneer 3dx en metros.
radioRueda = diametroRueda/2  #milimetros
# Numero de obstaculos
nObstacles = 0
# Arreglo con obstaculos
obstacles = []

# Es la distancia entre el punto P y el eje de cada rueda. ##TOCA MEDIRLO
l = 50  # milimetros
# Es la variable donde se almacena el valor de p (rho) que equivale a la distancia entre el punto actual y el final.
p = 0
# Es un umbral que se define para indicarle al robot cuando llega al punto final.
umbralP = 195 # distanciaCuadricula/2    ############# CAMBIO ###########
# Es una variable booleana que indica que el robot ha llegado al punto final.
arrivedP = False
# Es la variable donde se guarda la distancia en x entre el punto actual y final.
dx = 0
# Es la variable donde se guarda la distancia en y entre el punto actual y final.
dy = 0
# Es la variable donde se almacena el valor de a (alpha).
a = 0
# Es la variable donde se almacena el valor de b (beta).
b = 0
# Es la variable en donde se guarda el resultado de aplicar la funcion atan2 al triangulo formado por dy y dx.
# Equivale al angulo que se forma en el triangulo formado por el punto actual y final.
t = 0
# Es la constante kp. Debe ser mayor que 0 para que el sistema sea localmente estable.
kp = 0.3  #0.4 # mayor que 0, antes era 0.1
# Es la constante ka. ka-kp debe ser mayor que 0 para que el sistema sea localmente estable.
ka = 0.5  # 1 # ka-kp mayor que 0, antes era 0.5
# Es la constante kb. Debe ser menor a 0 para que el sistema sea localmente estable.
kb = 0.01  # menor que 0, era negativo -0.1
# Es la variable utilizada para publicar en el topico motorsVel la velocidad de cada motor.
mot = Float32MultiArray()
# En esta se almacenan las velocidades de cada motor.
mot.data = [0, 0]
# Senal para saber si ya se ejecuto
empezar = False

umbralFin = math.pi/6

# En este metodo se inicializa el nodo, se suscribe a los topicos necesarios, se crea la variable para publicar al
# topico de motorsVel y tambien se lanza el nodo encargado de graficar. Ademas es el metodo encargado de realizar
# las acciones de control necesarias segun la ruta dada para llevar el robot a la posicion final.
def control():
    global posicionActual, g, ruta, pubMot, arrivedP, p, umbralP, kp, kb, ka, empezar, pedal
    # Se crean el nodo
    rospy.init_node('nodoControl', anonymous=True)
    # Se suscribe a al topico de la informacion de la posicion del pioneer
    rospy.Subscriber ('robot_position', Pose, setPositionCallback)


    # Se crea referencia a topico para publicar velocidad de los motores
    pubMot = rospy.Publisher('desired_speed', Float32MultiArray, queue_size=10)
    # Se crea el servicio que provee el nodo para recibir parametros de informacion
    s = rospy.Service('iniciar_recorrido', StartService, handle_iniciar_recorrido)
    # Se espera a que se publique por primera vez a traves del topico preguntarCasillas
    while not empezar:
        pass



    # Se crean los vertices y casillas del grafo y arreglo respectivamente
    creadorVerticesCasillas()
    # Se crean los arcos del grafo
    creadorArcos()
    # Se emplea el metodo de la libreria networkx para sacar la ruta de nodos A*, a este metodo es necesario
    # suministrarle el nombre del nodo inicial, final y un parametro que haga referencia a un metodo que calcule la
    # heuristica entre dos nodos
    ruta = nx.astar_path(g, numCasillas(posicionActual.position.x, posicionActual.position.y),
                         numCasillas(posicionFinal.position.x, posicionFinal.position.y), heuristic=heuristic)
    #visualizacionPrevia(ruta)

    # En caso de que la ruta este compuesta por mas de un nodo calcula el teta adecuado para que la primera posicion
    # termine orientada a la siguiente casilla, de lo contrario, la orienta al punto final
    # Publica un 3 en topico estados
    if len(ruta) > 1:
        teta = math.atan2(casillas[ruta[1]].y-casillas[ruta[0]].y, casillas[ruta[1]].x-casillas[ruta[0]].x)
    else:
        teta = math.atan2(posicionFinal.position.y-casillas[ruta[0]].y, posicionFinal.position.x-casillas[ruta[0]].x)
    # Crea la primera posicion
    aux = Posicion(casillas[ruta[0]].x, casillas[ruta[0]].y, teta)

    posInter = Pose()
    posInter.position.x = aux.x
    posInter.position.y = aux.y
    posInter.orientation.w = aux.teta

    # Inicializa el contador que define el punto de la ruta en la que se encuentra
    iRuta = 0
    # Tasa a la que se debe publicar en el nodo del movimiento del pioneeer
    rate = rospy.Rate(10)
    # Variable booleana que define que se llego al punto final
    fin = False
    while (not rospy.is_shutdown()) and (not fin):
        # Entra al siguiente condicional en caso de que se halla llegado a uno de los puntos intermedios de la ruta
        if arrivedP:
            # Entra al siguiente condicional en caso de que halla llegado al punto final de la ruta, debe encaminarse a
            # posicion final del camino
            if iRuta == len(ruta)-1:
                iRuta = iRuta + 1
                arrivedP = False
                posInter = posicionFinal
                # Se modifica el umbral para que de igual forma llega mas cercano al punto final
                umbralP = 250#400   ######## CAMBIO ###############
            # Entra al siguiente condicional en caso de que halla llegado a un punto intermedio de la ruta
            elif iRuta < len(ruta)-1:
                aux = Posicion(casillas[ruta[iRuta+1]].x, casillas[ruta[iRuta+1]].y, math.atan2(casillas[ruta[iRuta+1]].y-casillas[ruta[iRuta]].y, casillas[ruta[iRuta+1]].x-casillas[ruta[iRuta]].x))
                posInter.position.x = aux.x
                posInter.position.y = aux.y
                posInter.orientation.w = aux.teta
                iRuta = iRuta + 1
                arrivedP = False
            # Entra de que halla llegado a la poscion final
            elif iRuta == len(ruta):
                # Debido a que llega a la posicon final se modifica el Kb para que modifique su orientacion a la final
                kb = -0.06
                # dif = abs(posicionFinal.orientation.w - posicionActual.orientation.w)
                # while dif > math.pi:
                    # dif = dif -math.pi*2
                # while dif < -math.pi:
                    # dif = dif + math.pi*2
                # if  dif < umbralFin:
                if comparadorAngulos(posicionFinal.orientation.w, posicionActual.orientation.w):

                    # En caso que la orientacion tenga un error menor a los 0.1 radianes en la poscion final termina
                    # procedimiento
                    fin = True
        # En cada iteracion calcula las velocidades segun el punto final que se le pase
        calcularVelocidades(posInter)
        #rospy.loginfo(posInter)
        # Publica en el topico la velocidad de los motores requerida
        pubMot.publish(mot)
        # Espera a que se cumpla la tasa de tiempo
        rate.sleep()

    # En caso de terminar detiene a el pioneer
    mot.data[0] = 0
    mot.data[1] = 0
    pubMot.publish(mot)
    # rospy.wait_for_service('terminar_control')
    terminar_control = rospy.ServiceProxy('terminar_control', TerminarRecorrido)
    # pubTermino = rospy.Publisher('termino_recorrido', Int32,  queue_size=10)
    # pubTermino.publish(1)
    terminar_control()
    rospy.spin()


def handle_iniciar_recorrido(startS):
    global posicionActual, posicionFinal, n_obstacles, obstacles, empezar
    posicionActual = startS.start
    posicionFinal = startS.goal
    rospy.loginfo(posicionActual)
    rospy.loginfo(posicionFinal)
    n_obstacles = startS.n_obstacles
    obstacles = startS.obstacles
    empezar = True
    return []


# Metodo que arroja la distancia euclidiana entre dos casillas segun su numeramiento (no posicion exacta)
def heuristic(i, j):
    global n 
    # Se retorna la distancia relativa entre casillas por distancia euclidiana
    return math.sqrt((i % n-j % n)**2+(i//n-j//n)**2)


# Busca la casilla mas cercana para dos coordenadas en la distribucion
def numCasillas(x, y):
    global n m
    # Inicializa la distancia minima en infinito y el indice de la casilla en -1
    dist = float('Inf')
    indice = -1
    # Se empieza a buscar la casilla mas cercana, en caso de que competidor sea mas cercano modifica la distancia minima
    # y el indice de la casilla
    for i in range(0, n * m):
        distancia = math.sqrt((casillas[i].x - x) ** 2 + (casillas[i].y - y) ** 2)
        if distancia < dist:
            indice = i
            dist = distancia
    return indice


# Metodo asociedo a topico para actualizar la posicon del pioneer
def setPositionCallback(pos):
    global posicionActual
    # Se modifica las cordedanadas de el objeto de posicion del pioneer
    posicionActual = pos


# Metodo que crea los vertices y casillas del arreglo y del grafo no dirigido
def creadorVerticesCasillas():
    global distanciaCuadricula, n , m
    # Las variables a continuacion definen desde que posicion se empieza a discretizar la escena
    xInic = distanciaCuadricula/2
    yInic = totaly - distanciaCuadricula/2 ########## CAMBIO ############
    # El ciclo empieza a generar la matriz de casillas donde se guarda la discretizacion y anade el nodo al grafo.
    for i in range(0, n*m):
        g.add_node(i)
        mod = i % n
        div= i//n
        nC = Casilla(xInic+mod*distanciaCuadricula, yInic-div*distanciaCuadricula, libre(xInic+mod*distanciaCuadricula, yInic-div*distanciaCuadricula))
        casillas.append(nC)

# Metodo que ejecuta nodo graficador usanto herramiento roslaunch, crea un nuevo proceso.
# def iniciarGraficador():
#     # Se define el paquete y nodo que se deben ejecutar
#     package = 'proyectoFinal'
#     script = 'graficador.py'
#     node = roslaunch.core.Node (package, script)
#     launch = roslaunch.scriptapi.ROSLaunch ()
#     launch.start ()
#     process = launch.launch (node)

def comparadorAngulos(angulo1, angulo2):
    an1 = angulo1
    an2 = angulo2
    while an1 > math.pi:
        an1 = an1 - 2*math.pi
    while an1 < -math.pi:
        an1 = an1 + 2*math.pi
    while an2 > math.pi:
        an2 = an2 - 2*math.pi
    while an2 < -math.pi:
        an2 = an2 + 2*math.pi
    dist = math.sqrt((math.cos(an1)-math.cos(an2))**2+(math.sin(an1)-math.sin(an2))**2)
    return dist < 0.25


# Crea los arcos entre los vertices creados en el grafo, solo es necesario revisar los nodos que pueden ser vecinos, en
# de que ambos esten vacios genera el arco.
def creadorArcos():
    global n m
    # El ciclo revisa cada uno de los nodos del grafo
    for i in range(0, n*m):
        # Se revisa primero si la casilla actual esta libre de obstaculos, de lo contrario no es necesario generar arcos
        if casillas[i].libre:
            c = i % n	# Eje X
            f = i//n # Eje Y
            # El ciclo revisa los posibles ocho vecinos de la casilla actual en cada una de las iteraciones
            for j in range(1, 9):
                if j == 1: # Left
                    cP = c-1
                    fP = f
                elif j == 2: # Down/Left
                    cP = c-1
                    fP = f-1
                elif j == 3: # Left
                    cP = c
                    fP = f-1
                elif j == 4: # Up/Left
                    cP = c+1
                    fP = f-1
                elif j == 5: # Up
                    cP = c+1
                    fP = f
                elif j == 6: # Up/Right
                    cP = c+1
                    fP = f+1
                elif j == 7: # Right
                    cP = c
                    fP = f+1
                elif j == 8: # Down/Right
                    cP = c-1
                    fP = f+1
                # En caso de que la casilla si pertenezca a los rangos de discretizacion y este libre se crea un arco
                # entre la casilla actual y el vecino revisado
                if cP in range(0, n) and fP in range(0, m) and casillas[fP*n+cP].libre:
                    g.add_edge(i, fP*n+cP)


# Metodo que indica si cierta posicion en la escena esta ocupada por alguno de los obstaculos, devuelve True en caso de
# que no haya obstaculo en ese punto y False de lo contrario
# preguntar distRef pero ya corregido
def libre(xCas, yCas):  # Si se encuentra un obstaculo en ella
    distanciaCarro = 200
    distBool = True
    for i in obstacles:
        rospy.loginfo("i:")
        rospy.loginfo(i)
        distObs = math.sqrt((i.position.position.x - xCas)**2 + (i.position.position.y - yCas)**2)
        distRef = i.radius + distanciaCarro
        distBool = (distObs >= distRef)
        if not distBool:
            break
    return distBool
    # En caso de que alguna distancia de referencia sea mayor que la distancia al obstaculo se considera que ese punto
    # esta ocupado por un obstaculo
    # return (dist0>=distRef0) and (dist1>=distRef1) and (dist2>=distRef2) and (dist3>=distRef3) and (dist4>=distRef4)


# Este metodo calcula la distancia p (rho) equivalente a la distancia entre el punto final y el actual y tambien calcula
# las distancias en el eje X y Y entre dichos puntos. Ademas, si p (rho) es menor al umbral definido se le indica al
# restodel codigo que se llego al punto final ademas ya eesta Pose
def calcularDistancia(pos):
    global p, dx, dy, arrivedP
    # Se define ro
    dx = pos.position.x-posicionActual.position.x
    dy = pos.position.y-posicionActual.position.y
    p = math.sqrt(dx**2 + dy**2)
    # Si ro es menor que el umbral se asume que ya se llego al punto adecuado y cambio la variable booleana de llegada
    if p <= umbralP:
        arrivedP = True


# Aqui se calculan las velocidades de los motores que mueven al robot.
def calcularVelocidades(pos):
    global v, w, mot
    # Se llama al metodo para actualizar ro
    calcularDistancia(pos)
    # se llama al metodo para actualizar alfa y beta
    calcularAngulos(pos)
    # se define la velocidad lineal y angular que se debe tener segun las constantes de sistema de control
    v = kp*p
    w = ka*a+kb*b
    # Se modifican las variables que se deben publicar en el topico
    mot.data[0] = (v-l*w)
    mot.data[1] = (v+l*w)


# En este metodo se calculan los angulos a (alpha) y b (beta). Si el umbral se cumplio, las variables p (rho)
# y a (alpha) se igualan a 0 para permitirle al robot girar y lograr orientarse de manera correcta. Ya en z
def calcularAngulos(pos):
    global p, a, b, t
    t = math.atan2(dy, dx)
    # Se define si alfa y ro deben ser cero o no dependiendo de si se llego al umbral
    if not arrivedP:
        a = -posicionActual.orientation.w + t
    else:
        p = 0
        a = 0
    # Se restringe el valor de alfa entre -pi y pi
    if a > math.pi:
        while a > math.pi:
            a = a - 2 * math.pi
    elif a <= -math.pi:
        while a <= -math.pi:
            a = a + 2 * math.pi
    # Se restringe el valor de beta entre -pi y pi
    b = posicionActual.orientation.w-pos.orientation.w - a
    if b > math.pi:
        while b > math.pi:
            b = b - 2 * math.pi
    elif b <= -math.pi:
        while b <= -math.pi:
            b = b + 2 * math.pi


# Metodo que grafica el camino generado por el RTT previo a que se realice la accion de control
def visualizacionPrevia(path):
    global g
    cordX = []
    cordY = []
    cordXObs = []
    cordYObs = []
    xPath = []
    yPath = []

    for i in range(0, len(casillas)):
        casillaActual = casillas[i]
        if casillaActual.libre:
            if i in path:
                xPath.append(casillaActual.x)
                yPath.append(casillaActual.y)
            else:
                cordX.append(casillaActual.x)
                cordY.append(casillaActual.y)
        else:
            if i not in path:
                cordXObs.append (casillaActual.x)
                cordYObs.append (casillaActual.y)
            else:
                print("Error casilla ocupada pertenece a ruta")
    plt.plot(cordX, cordY, 'bo')
    plt.plot(cordXObs, cordYObs, 'ro')
    plt.plot(xPath, yPath, 'go')
    plt.show()


# Metodo main, mira si existen parametros para la posicion final deseada y ejecuta el metodo principal
if __name__ == '__main__':
    try:
        # En caso de que se pasen tres parametros tipo numero se ajusta la nueva posicion final deseada
        control()
    except rospy.ROSInterruptException:
        pass
