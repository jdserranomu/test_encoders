#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import time
import math
import sys
from std_msgs.msg import Float32MultiArray

# Es la tasa en Hertz (Hz) del nodo.
h = 10
# Variable con el primer pin que va al driver para controlar el motor A.
pwmA1Driver = 11
# Variable con el segundo pin que va al driver para controlar el motor A.
pwmA2Driver = 12
# Variable con el primer pin que va al driver para controlar el motor B.
pwmB1Driver = 15
# Variable con el segundo pin que va al driver para controlar el motor B.
pwmB2Driver = 16
# Frecuencia en Hertz (Hz) de la senal de pulso que controla el motor.
freq_driver = 500
# Variable con el pin que va del encoder con la senal A1
pwmA1Encoder = 35
# Variable con el pin que va del encoder con la senal B1
pwmB1Encoder = 36
# Variable con el pin que va del encoder con la senal A2
pwmA2Encoder = 37
# Variable con el pin que va del encoder con la senal B2
pwmB2Encoder = 38
# radio de la rueda
radioRueda = 14.65  # milimetros
# Variables que referencian senales PMW de los encoders
pA1 = None
pA2 = None
pB1 = None
pB2 = None
# Variables de conteo de flancos de encoders
contadorA = 0
contadorB = 0
# Referencia de valor de contador de ultima vez que se calculo la velocidad
refContadorA = 0
refContadorB = 0
# Referencia de valor de tiempo de ultima vez que se calculo la velocidad
refTiempo = 0
# Referencia de seguridad de accion de control para cuando pase por 0
refAccionControlA = 0
refAccionControlB = 0
# Referencia de pwm de viejo en ruedas
pwmA = 0
pwmB = 0
# Variable booleana  que inidca si se esta calculando velocidad
calculando = False
# Radio de las llantas en metros
r = (29.3/2)
# Variables de control PI
kpA = 0.3 #Antes 0.2
kiA = 63.4
kdA = 0.0009
kpB = 0.1 #Antes 0.2
kiB = 76.4
kdB = 0.0016
# Acumulacion de error para integrador
integradorA = []
integradorB = []
# Velocidades de referencia de las ruedas // ASUMIRE rueda A izquierda (vel positiva counter clockwise, B antes que A) y
# rueda B derecha (vel positiva clockwise, A antes que B)
velRefA = 0
velRefB = 0
# Velocidades actual de las ruedas
velActA = 0
velActB = 0
pubVelsAct = Float32MultiArray()
# Variable de saturacion maxima de ciclo util
satCicloUtil = 60

errorAnteriorA = 0
errorAnteriorB = 0


def setPins():
    global pA1, pA2, pB1, pB2
    # Configurandp estructura de pins de raspberry
    GPIO.setmode(GPIO.BOARD)
    # Configurando los pines de salida para el driver
    GPIO.setup(pwmA1Driver, GPIO.OUT)
    GPIO.setup(pwmA2Driver, GPIO.OUT)
    GPIO.setup(pwmB1Driver, GPIO.OUT)
    GPIO.setup(pwmB2Driver, GPIO.OUT)
    # Configurando pines de salida para los encoders
    GPIO.setup(pwmA1Encoder, GPIO.IN)
    GPIO.setup(pwmB1Encoder, GPIO.IN)
    GPIO.setup(pwmA2Encoder, GPIO.IN)
    GPIO.setup(pwmB2Encoder, GPIO.IN)
    # Configurando senales de salida para el driver e inicializandolas en ciclo util de 0
    pA1 = GPIO.PWM(pwmA1Driver, freq_driver)
    GPIO.output(pwmA1Driver, 0)
    pA2 = GPIO.PWM(pwmA2Driver, freq_driver)
    GPIO.output(pwmA2Driver, 0)
    pB1 = GPIO.PWM(pwmB1Driver, freq_driver)
    GPIO.output(pwmB1Driver, 0)
    pB2 = GPIO.PWM(pwmB2Driver, freq_driver)
    GPIO.output(pwmB2Driver, 0)
    # Detectar flancos en otros metodos
    GPIO.add_event_detect(pwmA1Encoder, GPIO.BOTH, callback=sumarFlancoA1)
    GPIO.add_event_detect(pwmB1Encoder, GPIO.BOTH, callback=sumarFlancoB1)
    GPIO.add_event_detect(pwmA2Encoder, GPIO.BOTH, callback=sumarFlancoA2)
    GPIO.add_event_detect(pwmB2Encoder, GPIO.BOTH, callback=sumarFlancoB2)


def controlBajoNivel():
    global pub, pubVelsAct
    rospy.init_node('low_level_controller', anonymous=True)

    rospy.Subscriber('desired_velocity', Float32MultiArray, handle_velocidad_deseada)
    pub = rospy.Publisher('actual_velocity', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(h)
    setPins()
    while not rospy.is_shutdown():
        time = calcularVelocidadRuedas()
        aplicarControlBajoNivel(time)
        pubVelsAct.data = [velActA, velActB]
        pub.publish(pubVelsAct)


        rate.sleep()
    apagar()



def calcularVelocidadRuedas():
    global velActA, velActB, refContadorA, refContadorB, refTiempo
    flancosA = contadorA - refContadorA
    flancosB = contadorB - refContadorB
    tiempoNuevo = float(time.time())
    tiempo = tiempoNuevo - refTiempo
    refContadorA = contadorA
    refContadorB = contadorB
    refTiempo = tiempoNuevo
    velActA = (flancosA/tiempo)*(math.pi/72)*radioRueda
    velActB = (flancosB/tiempo)*(math.pi/72)*radioRueda
    return tiempo


def aplicarControlBajoNivel(time):
    global integradorA, integradorB, pA1, pA2, pB1, pB2, refAccionControlA, refAccionControlB, pwmA, pwmB, errorAnteriorA, errorAnteriorB
    errorA = velRefA - velActA
    errorB = velRefB - velActB
    integradorA.append(errorA)
    integradorA = integradorA[-1000:]
    integradorB.append(errorB)
    integradorB = integradorB[-1000:]
    integralA = sum(integradorA) * time
    integralB = sum(integradorB) * time
    derivadaErrorA = (errorA-errorAnteriorA)/time
    derivadaErrorB = (errorB-errorAnteriorB)/time
    errorAnteriorA = errorA
    errorAnteriorB = errorB

    pwmA = kpA * errorA + kiA * integralA + kdA * derivadaErrorA
    pwmB = kpB * errorB + kiB * integralB + kdB * derivadaErrorB

    if velRefA == 0 or abs(pwmA) < 1:
        pwmA = 0
    if velRefB == 0 or abs(pwmB) < 1:
        pwmB = 0
    if pwmA >= 0:
        if pwmA > satCicloUtil:
            pwmA = satCicloUtil
        if refAccionControlA < 0:
            pwmA = 0
        pA2.stop()
        GPIO.output(pwmA2Driver, 0)
        pA1.start(0)
        pA1.ChangeDutyCycle(abs(pwmA))
    else:
        if pwmA < -satCicloUtil:
            pwmA = -satCicloUtil
        if refAccionControlA > 0:
            pwmA = 0
        pA1.stop ()
        GPIO.output (pwmA1Driver, 0)
        pA2.start (0)
        pA2.ChangeDutyCycle (abs(pwmA))
    if pwmB >= 0:
        if pwmB > satCicloUtil:
            pwmB = satCicloUtil
        if refAccionControlB < 0:
            pwmB = 0
        pB1.stop()
        GPIO.output(pwmB1Driver, 0)
        pB2.start(0)
        pB2.ChangeDutyCycle(abs(pwmB))
    else:
        if pwmB < -satCicloUtil:
            pwmB = -satCicloUtil
        if refAccionControlB > 0:
            pwmB = 0
        pB2.stop()
        GPIO.output(pwmB2Driver, 0)
        pB1.start(0)
        pB1.ChangeDutyCycle(abs(pwmB))
    refAccionControlA = pwmA
    refAccionControlB = pwmB


def handle_velocidad_deseada(vel):
    global velRefA, velRefB
    velRefA = vel.data[0]
    velRefB = vel.data[1]


def apagar():
    global pwmA, pwmB
    pwmA = 0
    pwmB = 0
    GPIO.output(pwmA1Driver, 0)
    GPIO.output(pwmA2Driver, 0)
    GPIO.output(pwmB1Driver, 0)
    GPIO.output(pwmB2Driver, 0)
    GPIO.cleanup()


def sumarFlancoA1(channel):
    global contadorA
    if GPIO.input(pwmA1Encoder):
        if GPIO.input(pwmB1Encoder):
            contadorA = contadorA + 1
        else:
            contadorA = contadorA - 1
    else:
        if GPIO.input(pwmB1Encoder):
            contadorA = contadorA - 1
        else:
            contadorA = contadorA + 1


def sumarFlancoB1(channel):
    global contadorA
    if GPIO.input(pwmB1Encoder):
        if GPIO.input(pwmA1Encoder):
            contadorA = contadorA - 1
        else:
            contadorA = contadorA + 1
    else:
        if GPIO.input(pwmA1Encoder):
            contadorA = contadorA + 1
        else:
            contadorA = contadorA - 1


def sumarFlancoA2(channel):
    global contadorB
    if GPIO.input(pwmA2Encoder):
        if GPIO.input(pwmB2Encoder):
            contadorB = contadorB - 1
        else:
            contadorB = contadorB + 1
    else:
        if GPIO.input (pwmB2Encoder):
            contadorB = contadorB + 1
        else:
            contadorB = contadorB - 1


def sumarFlancoB2(channel):
    global contadorB
    if GPIO.input (pwmB2Encoder):
        if GPIO.input (pwmA2Encoder):
            contadorB = contadorB + 1
        else:
            contadorB = contadorB - 1
    else:
        if GPIO.input (pwmA2Encoder):
            contadorB = contadorB - 1
        else:
            contadorB = contadorB + 1


if __name__ == '__main__':
    try:
        if len(sys.argv) == 3:
            try:
                velRefA = float(sys.argv[1])
                velRefB = float(sys.argv[2])
                print("Velocidad inicial rueda A:" + str(velRefA), "Velocidad inicial rueda B:" + str(velRefB))
            except ValueError:
                pass
        controlBajoNivel()
    except rospy.ROSInterruptException:
        pass