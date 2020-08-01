#!/usr/bin/env python
import rospy
import socket
import time
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Pose
#from master_msgs_iele3338.msg import Obstacle
#from master_msgs_iele3338.srv import StartService, EndService
from test_encoders.srv import StartService, TerminarRecorrido, StartServiceRequest
from test_encoders.msg import Obstacle

import os

#Variable con toda la informacion del escenario
escenario = StartServiceRequest()
# Posicion de inicio del robot
start = Pose()
# Posicion final a la que se quiere llegar
goal = Pose()
# Numero de obstaculos que se envian
n_obstacles = 0
# Objeto tipo Obstacle con informacion de los obstaculos
obstacles_array = []
# Contrasena obtenida
password = None
# Variable que identifica si se termino el recorrido
esperarTerminarRecorrido = False
# Variable que identifica que se solicito el servicio start_service
esperarStartService = False

positionsMap=[]

def createMap():
    tam=250
    cte=500
    global positionsMap
    for i in range(4):
        pos=Pose()
        pos.position.x=cte*i+tam/2
        pos.position.y=tam/2
        pos.orientation.w=0
        positionsMap.append(pos)
    for i in range(4):
        pos=Pose()
        pos.position.x=cte*i+tam+tam/2
        pos.position.y=1000-tam/2
        pos.orientation.w=0
        positionsMap.append(pos)
def leviathan():
    createMap()
    rospy.init_node('nodoPrincipal', anonymous=True)  # inicializa el nodo


    try:

        start_service(1,7,0,[])
        rospy.wait_for_service('iniciar_recorrido')
        rospy.wait_for_service('iniciar_odometria')
        rospy.wait_for_service('iniciar_encoders')

        iniciar_service = rospy.ServiceProxy('iniciar_recorrido', StartService)  # Crea el objeto referente al servicio

        iniciar_service(escenario)
        iniciar_odometria = rospy.ServiceProxy('iniciar_odometria', StartService)
        iniciar_odometria(escenario)
        iniciar_encoders = rospy.ServiceProxy('iniciar_encoders', StartService)
        iniciar_encoders(escenario)
        # rospy.Subscriber ('termino_recorrido', Int32, handle_terminar_recorrido)
        s1=rospy.Service('terminar_control', TerminarRecorrido, handle_terminar_recorrido)
        rospy.loginfo("Esperando terminar control")
        while not esperarTerminarRecorrido:
            pass




    except rospy.ServiceException:
        print("Ocurrio un error solicitando servicio")


def start_service(start,end,n_obs,obs):
    global positionsMap,escenario
 
    startS=StartServiceRequest()
    startS.start=positionsMap[start]
    startS.goal=positionsMap[end]
    startS.n_obstacles=n_obs
    startS.obstacles=obs

    escenario = startS
    start = startS.start
    goal = startS.goal
    n_obstacles = startS.n_obstacles

    obstacles_array = startS.obstacles


    return []


def handle_terminar_recorrido(req):
    global esperarTerminarRecorrido
    esperarTerminarRecorrido = True
    rospy.loginfo ("Recibio el llamado del topico para terminar recorrido")
    return []





if __name__ == '__main__':
    try:
        leviathan()
    except rospy.ROSInterruptException:
        pass
