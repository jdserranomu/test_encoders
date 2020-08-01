#!/usr/bin/env python
#Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy, math, roslaunch, time
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from test_encoders.msg import Covariance
import numpy as np
from test_encoders.srv import StartService

pos = Pose()
cov = Covariance()

#El primero es la velocidad de la rueda izquierda y el segundo es la velocidad de la derecha.
vel = [0, 0]

#Son las constantes utilizadas para calcular la posicion y orientacion estimada del robot en cada momento.
dt = 0
dSl = 0
dSr = 0
dS = 0
dScos=0
dSsin=0
cose=0
seno=0

#Averiguar que es b. Por ahora en 1.
b = float(2*50)

#Constantes de la matriz de covarianza. Por ahora en 1.
kr = 0.1
kl = 0.1

#Theta. Es la orientacion del robot con respecto al eje z.
O = 0
dO = 0

#Es la tasa en Hz del nodo.
h = 10

#Covarianza
Covarianza=np.zeros((3,3))
CovarSrSl=np.zeros((2,2))

Fpt1=np.zeros((3,3))
Fpt1trans=np.transpose(Fpt1)
FdS=np.zeros((3,2))
FdStrans=np.transpose(Fpt1)

#  En esta variable se almacenan los ultimos 2 valores de tiempo.
t = [0, 0]

#  Indica cuando comenzar
empezar = False


def odometria():
    #  rospy.wait_for_service()
    rospy.init_node('nodo_odometria', anonymous = True)
    rate = rospy.Rate(10)
    s = rospy.Service('iniciar_odometria', StartService, handle_iniciar_odometria)
    # Se espera a que se publique por primera vez a traves del topico preguntarCasillas
    while not empezar:
        pass
    inicializar()

    while not rospy.is_shutdown():
        rate.sleep()


def inicializar():
    global pubPos, pubCov, t
    pubPos = rospy.Publisher('robot_position', Pose, queue_size = 10)
    pubCov = rospy.Publisher('robot_uncertainty', Covariance, queue_size = 10)
    rospy.Subscriber('current_speed', Float32MultiArray, actualizar)
    t = [time.time(), time.time()]

def actualizar(msg):
    global pos, cov, vel, t, dt, dS, dSl, dSr, O, dO, Covarianza, pubPos, pubCov, CovarSrSl, CovarSrSl, Fpt1, Fpt1trans, FdS, FdStrans, cose, seno, dScos, dSsin
    t.append(time.time())
    t = t[-2:]
    dt = (t[1]-t[0])
    dSl = vel[0]*dt
    dSr = vel[1]*dt
    dS = (dSl+dSr)/2
    dO = (dSr-dSl)/b
    O = pos.orientation.w
    cose = math.cos(O+dO/2.0)
    seno = math.sin(O+dO/2.0)
    dScos = dS*cose
    dSsin = dS*seno
    pos.position.x = pos.position.x + dScos
    pos.position.y = pos.position.y + dSsin
    pos.orientation.w = O + dO
    while pos.orientation.w < -math.pi:
        pos.orientation.w = pos.orientation.w + 2*math.pi
    while pos.orientation.w > math.pi:
        pos.orientation.w = pos.orientation.w - 2*math.pi
    CovarSrSl = np.array([[kr*np.absolute(dSr), 0], [0, kl*np.absolute(dSl)]])
    Fpt1 = np.array([[1.0, 0, -dSsin], [1.0, 0, dScos], [0, 0, 1.0]])
    Fpt1trans = Fpt1.transpose()
    FdS = np.array([[(1.0/2.0)*cose-(1.0/(2.0*b))*dSsin, (1.0/2.0)*cose+(1.0/(2.0*b))*dSsin], [(1.0/2.0)*seno+(1.0/(2.0*b))*dScos, (1.0/2.0)*seno-(1.0/(2.0*b))*dScos], [1.0/b, -(1.0/b)]])
    FdStrans = FdS.transpose()
    Covarianza = np.dot((np.dot(Fpt1, Covarianza)), Fpt1trans) + np.dot(np.dot(FdS, CovarSrSl), FdStrans)
    cov.sigma11 = Covarianza[0, 0]
    cov.sigma12 = Covarianza[0, 1]
    cov.sigma13 = Covarianza[0, 2]
    cov.sigma21 = Covarianza[1, 0]
    cov.sigma22 = Covarianza[1, 1]
    cov.sigma23 = Covarianza[1, 2]
    cov.sigma31 = Covarianza[2, 0]
    cov.sigma32 = Covarianza[2, 1]
    cov.sigma33 = Covarianza[2, 2]
    pubPos.publish(pos)
    pubCov.publish(cov)
    vel = msg.data


def handle_iniciar_odometria(startS):
    global pos, empezar
    empezar = True

    pos = startS.start
    rospy.loginfo("Handle")
    return []

if __name__ == '__main__':
    try:
        odometria()
    except rospy.ROSInterruptException:
        pass
