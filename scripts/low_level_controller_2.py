#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import time
import math
import sys
from std_msgs.msg import Float32MultiArray

# Es la tasa en Hertz (Hz) del nodo.
freq = 10
# Variable con el primer pin que va al driver para controlar el motor A.
pinDriverA1 = 11
# Variable con el segundo pin que va al driver para controlar el motor A.
pinDriverA2 = 12
# Variable con el primer pin que va al driver para controlar el motor B.
pinDriverB1 = 15
# Variable con el segundo pin que va al driver para controlar el motor B.
pinDriverB2 = 16
# Frecuencia en Hertz (Hz) de la senal de pulso que controla el motor.
freq_driver = 500
# Ciclo util del pulso para el motor A. Un numero entre 0 y 100.
dutyCycleDriverA = 0
# Ciclo util del pulso para el motor B. Un numero entre 0 y 100.
dutyCycleDriverB = 0
# Variable con el pin que va del encoder con la senal A1
pinEncoderA1 = 35
# Variable con el pin que va del encoder con la senal B1
pinEncoderB1 = 36
# Variable con el pin que va del encoder con la senal A2
pinEncoderA2 = 37
# Variable con el pin que va del encoder con la senal B2
pinEncoderB2 = 38
# radio de la rueda
radiusWheel = 14.65  # mm
# Variables que referencian senales PMW de los drivers
pwmDriverA1 = None
pwmDriverA2 = None
pwmDriverB1 = None
pwmDriverB2 = None
# Variables de conteo de flancos de encoders
counter1 = 0
counter2 = 0
# Referencia de valor de contador de ultima vez que se calculo la velocidad
referenceCounter1 = 0
referenceCounter2 = 0
# Referencia de valor de tiempo de ultima vez que se calculo la velocidad
referenceTime = 0
# Referencia de seguridad de accion de control para cuando pase por 0
referenceControlActionA = 0
referenceControlActionB = 0
# Referencia de pwm de viejo en ruedas
dutyCyclePwmA = 0
dutyCyclePwmB = 0
# Variables de control PI
kpA = 0.2 #Antes 0.2
kiA = 0.000001
kdA = 0.01
kpB = 0.4 #Antes 0.2
kiB = 0.000001
kdB = 0.01
# Acumulacion de error para integrador
integrandA = []
integrandB = []
# Velocidades de referencia de las ruedas
desiredSpeedA = 0
desiredSpeedB = 0
# Velocidades actual de las ruedas
currentSpeedA = 0
currentSpeedB = 0
# Instancia para reutilizar de msnaje a topico de velocidad actual
messageCurrentSpeedPublisher = Float32MultiArray()
# Variable de saturacion maxima de ciclo util
dutyCycleSaturation = 70
# Ultimo error de la señal de velocidad
lastErrorA = 0
lastErrorB = 0

def setPins():
    global pwmDriverA1, pwmDriverA2, pwmDriverB1, pwmDriverB2
    # Configurandp estructura de pins de raspberry
    GPIO.setmode(GPIO.BOARD)
    # Configurando los pines de salida para el driver
    GPIO.setup(pinDriverA1, GPIO.OUT)
    GPIO.setup(pinDriverA2, GPIO.OUT)
    GPIO.setup(pinDriverB1, GPIO.OUT)
    GPIO.setup(pinDriverB2, GPIO.OUT)
    # Configurando pines de salida para los encoders
    GPIO.setup(pinEncoderA1, GPIO.IN)
    GPIO.setup(pinEncoderB1, GPIO.IN)
    GPIO.setup(pinEncoderA2, GPIO.IN)
    GPIO.setup(pinEncoderB2, GPIO.IN)
    # Configurando senales de salida para el driver e inicializandolas en ciclo util de 0
    pwmDriverA1 = GPIO.PWM(pinDriverA1, freq_driver)
    GPIO.output(pinDriverA1, 0)
    pwmDriverA2 = GPIO.PWM(pinDriverA2, freq_driver)
    GPIO.output(pinDriverA2, 0)
    pwmDriverB1 = GPIO.PWM(pinDriverB1, freq_driver)
    GPIO.output(pinDriverB1, 0)
    pwmDriverB2 = GPIO.PWM(pinDriverB2, freq_driver)
    GPIO.output(pinDriverB2, 0)
    # Detectar flancos en otros metodos
    GPIO.add_event_detect(pinEncoderA1, GPIO.BOTH, callback=flankA1)
    GPIO.add_event_detect(pinEncoderB1, GPIO.BOTH, callback=flankB1)
    GPIO.add_event_detect(pinEncoderA2, GPIO.BOTH, callback=flankA2)
    GPIO.add_event_detect(pinEncoderB2, GPIO.BOTH, callback=flankB2)


def controlBajoNivel():
    global messageCurrentSpeedPublisher
    rospy.init_node('low_level_controller', anonymous=True)
    rospy.Subscriber('desired_speed', Float32MultiArray, handleDesiredSpeed)
    pub = rospy.Publisher('current_speed', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(freq)
    setPins()
    while not rospy.is_shutdown():
        delta_time = measureWheelsSpeed()
        applyLowLevelControl(delta_time)
        messageCurrentSpeedPublisher.data = [currentSpeedB, currentSpeedB]
        pub.publish(messageCurrentSpeedPublisher)
        rate.sleep()
    shutdown()


def applyLowLevelControl(time):
    global integrandA, integrandB, pwmDriverA1, pwmDriverA2, pwmDriverB1, pwmDriverB2, referenceControlActionA, referenceControlActionB, dut, pwmB, errorAnteriorA, errorAnteriorB
    errorA = velRefA - velActA
    errorB = velRefB - velActB
    integradorA.append(errorA)
    integradorA = integradorA[-100:]
    integradorB.append(errorB)
    integradorB = integradorB[-100:]
    integralA = sum(integradorA) * time
    integralB = sum(integradorB) * time
    derivadaErrorA = (errorA-errorAnteriorA)/time
    derivadaErrorB = (errorB-errorAnteriorB)/time
    errorAnteriorA = errorA
    errorAnteriorB = errorB
    # errorSignalA = kpA * errorA + kiA * integralA + kdA * derivadaErrorA
    # errorSignalB = kpB * errorB + kiB * integralB + kdB * derivadaErrorB
    # if abs(errorSignalA) < .1:
        # errorSignalA = 0
    # if abs(errorSignalB) < .1:
        # errorSignalB = 0
    pwmA = kpA * errorA + kiA * integralA + kdA * derivadaErrorA
    pwmB = kpB * errorB + kiB * integralB + kdB * derivadaErrorB
    # pwmA = pwmA + errorSignalA
    # pwmB = pwmB + errorSignalB
    # pwmA = velActA * (10/(math.pi*radioRueda))+errorSignalA
    # pwmB = velActB * (10/(math.pi*radioRueda))+errorSignalB
    if velRefA == 0:
        pwmA = 0
    if velRefB == 0:
        pwmB = 0
    if pwmA >= 0:
        if pwmA > satCicloUtil:
            pwmA = satCicloUtil
        if refAccionControlA < 0:
            pwmA = 0
        pA2.stop()
        GPIO.output (pwmA2Driver, 0)
        pA1.start (0)
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
        if refAccionControlB<0:
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


def measureWheelsSpeed():
    global currentSpeedA, currentSpeedB, referenceCounter1, referenceCounter2, referenceTime
    delta_flanksA = counter1 - referenceCounter1
    delta_flanksB = counter2 - referenceCounter2
    new_time_reference = float(time.time())
    delta_time = new_time_reference - referenceTime
    referenceCounter1 = counter1
    referenceCounter2 = counter2
    referenceTime = new_time_reference
    currentSpeedA = (delta_flanksA/delta_time)*(math.pi/600)*radiusWheel
    currentSpeedB = (delta_flanksB/delta_time)*(math.pi/600)*radiusWheel
    return delta_time


def shutdown():
    global dutyCyclePwmA, dutyCyclePwmB
    dutyCyclePwmA = 0
    dutyCyclePwmB = 0
    GPIO.output(pinDriverA1, 0)
    GPIO.output(pinDriverA2, 0)
    GPIO.output(pinDriverB1, 0)
    GPIO.output(pinDriverB2, 0)
    GPIO.cleanup()


def handleDesiredSpeed(speed):
    global desiredSpeedA, desiredSpeedB
    desiredSpeedA = speed.data[0]
    desiredSpeedB = speed.data[1]


def flankA1(channel):
    global counter1
    if GPIO.input(pinEncoderA1):
        if GPIO.input(pinEncoderB1):
            counter1 += 1
        else:
            counter1 -= 1
    else:
        if GPIO.input(pinEncoderB1):
            counter1 -= 1
        else:
            counter1 += 1


def flankB1(channel):
    global counter1
    if GPIO.input(pinEncoderB1):
        if GPIO.input(pinEncoderA1):
            counter1 -= 1
        else:
            counter1 += 1
    else:
        if GPIO.input(pinEncoderA1):
            counter1 += 1
        else:
            counter1 -= 1


def flankA2(channel):
    global counter2
    if GPIO.input(pinEncoderA2):
        if GPIO.input(pinEncoderB2):
            counter2 -= 1
        else:
            counter2 += 1
    else:
        if GPIO.input (pinEncoderB2):
            counter2 += 1
        else:
            counter2 -= 1


def flankB2(channel):
    global counter2
    if GPIO.input(pinEncoderB2):
        if GPIO.input(pinEncoderA2):
            counter2 += 1
        else:
            counter2 -= 1
    else:
        if GPIO.input(pinEncoderA2):
            counter2 -= 1
        else:
            counter2 += 1


if __name__ == '__main__':
    try:
        if len(sys.argv) == 3:
            try:
                desiredSpeedA = float(sys.argv[1])
                desiredSpeedB = float(sys.argv[2])
                print("Velocidad inicial rueda A:" + str(desiredSpeedA), "Velocidad inicial rueda B:" + str(desiredSpeedB))
            except ValueError:
                pass
        lowLevelController()
    except rospy.ROSInterruptException:
        pass