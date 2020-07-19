#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import time
import math
import sys
import csv

# Es la tasa en Hertz (Hz) del nodo.
h = 50
# Variable con el primer pin que va al driver para controlar el motor A.
pin_driver1 = 32
# Variable con el segundo pin que va al driver para controlar el motor A.
pin_driver2 = 33
# Frecuencia en Hertz (Hz) de la senal de pulso que controla el motor.
fDriver = 500
# Ciclo util del pulso para el motor A. Un numero entre 0 y 100.
duty_cycle = 0
# Variable con el pin que va del encoder con la senal A1
pin_encoderA = 15
# Variable con el pin que va del encoder con la senal B1
pin_encoderB = 16
# Variables de conteo de flancos de encoders
counter = 0
# Variable referencia contador ultimo calculo
reference_counter = 0
reference_time = 0
# Variable de saturacion maxima de ciclo util
duty_cycle_sat = 60
# Radio rueda en mm
r = (29.3/2)


def set_pins():
    global reference_time
    # Configurandp estructura de pins de raspberry
    GPIO.setmode(GPIO.BOARD)
    # Configurando los pines de salida para el driver
    GPIO.setup(pin_driver1, GPIO.OUT)
    GPIO.setup(pin_driver2, GPIO.OUT)
    # Configurando pines de salida para los encoders
    GPIO.setup(pin_encoderA, GPIO.IN)
    GPIO.setup(pin_encoderB, GPIO.IN)
    # Configurando senales de salida para el driver e inicializandolas en ciclo util de 0
    # Detectar flancos en otros metodos
    reference_time = time.time()
    GPIO.add_event_detect(pin_encoderA, GPIO.BOTH, callback=flank_A)
    GPIO.add_event_detect(pin_encoderB, GPIO.BOTH, callback=flank_B)


def test_driver_encoder():
    rospy.init_node('test_driver_encoder', anonymous=True)
    rate = rospy.Rate(h)
    set_pins()
    if duty_cycle >= 0:
        pwm_driver = GPIO.PWM(pin_driver1, fDriver)
        # pwm_driver.start(abs(duty_cycle))
    else:
        pwm_driver = GPIO.PWM(pin_driver2, fDriver)
        # pwm_driver.start(abs(duty_cycle))
    data = [["time (s)", "speed (mm/s)", "duty cycle (%)"]]
    start_time = time.time()
    actual_duty_cycle=0
    while not rospy.is_shutdown():
        if time.time()-start_time >= 10:
            pwm_driver.start(abs(duty_cycle))
            actual_duty_cycle = duty_cycle
        speed = measure_speed()
        data.append([reference_time-start_time, speed, actual_duty_cycle])
        # print("Speed: " + str(speed) + " mm/s")
        rate.sleep()
    # print(data[0:3])
    with open('data.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(data)
    print("Stopped")
    pwm_driver.stop()
    shutdown()


def measure_speed():
    global reference_counter, reference_time
    diff_flanks = counter - reference_counter
    new_time = float(time.time())
    diff_time = new_time - reference_time
    reference_counter = counter
    reference_time = new_time
    speed = (diff_flanks/diff_time)*(math.pi/600)*r
    return speed


def shutdown():
    GPIO.output(pin_driver1, 0)
    GPIO.output(pin_driver2, 0)
    GPIO.cleanup()


def flank_A(channel):
    global counter
    if GPIO.input(pin_encoderA):
        if GPIO.input(pin_encoderB):
            counter += 1
        else:
            counter -= 1
    else:
        if GPIO.input(pin_encoderB):
            counter -= 1
        else:
            counter += 1


def flank_B(channel):
    global counter
    if GPIO.input(pin_encoderB):
        if GPIO.input(pin_encoderA):
            counter -= 1
        else:
            counter += 1
    else:
        if GPIO.input(pin_encoderA):
            counter += 1
        else:
            counter -= 1


if __name__ == '__main__':
    try:
        if len(sys.argv) == 2:
            try:
                duty_cycle = float(sys.argv[1])
                if duty_cycle < -duty_cycle_sat or duty_cycle > duty_cycle_sat:
                    print("Argument out o bounds")
                    duty_cycle = 10
            except ValueError:
                print("Error in arguments")
                duty_cycle = 10
        else:
            duty_cycle = 10
        test_driver_encoder()
    except rospy.ROSInterruptException:
        pass
