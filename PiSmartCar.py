
from picar import back_wheels
from picar import front_wheels
from SunFounder_Ultrasonic_Avoidance import Ultrasonic_Avoidance
from threading import Thread
from ubidots import ApiClient
import datetime
import picar
import random
import RPi.GPIO as GPIO
import sys
import time

# Constants.
COLOR_NONE    = 0
COLOR_RED     = 1
COLOR_GREEN   = 2
COLOR_ORANGE  = 3
DISTANCE_STOP = 15
DISTANCE_TURN = 25
FORCE_TURNING = 0
PIN_BUZZER    = 16       # BCM pin 16
PIN_LED       = (12, 26) # BCM pins 12, 26.
SPEED_BW      = 70
SPEED_FW_MAX  = 75
SPEED_FW_MIN  = 65
MAX_TRIES     = 10

# Globals.
enable_mqtt   = True
last_angle    = 90
last_dir      = 0
speed_fw      = 65
speed_limit   = 70

#
# setup
# Setup PiCar, GPIO pins and Ubidots.
#
def setup():
    global api, bw, fw, buzzer, led_green, led_red, mqtt_distance, mqtt_speed, ua

    # 1. PiCar setup.
    picar.setup()
    ua = Ultrasonic_Avoidance.Ultrasonic_Avoidance(20)
    fw = front_wheels.Front_Wheels(db='config')
    bw = back_wheels.Back_Wheels(db='config')
    fw.turning_max = 45

    # 2. GPIO pins for buzzer and LED.
    GPIO.setmode(GPIO.BCM)           # GPIO.BCM mode.
    GPIO.setup(PIN_BUZZER, GPIO.OUT) # Buzzer pin.
    GPIO.setup(PIN_LED, GPIO.OUT)    # LED pins.
    GPIO.output(PIN_LED, GPIO.LOW)   # Set LED pins to low to turn off LEDs.
    buzzer    = GPIO.PWM(PIN_BUZZER, 440)  # Set buzzer frequency to 440Hz.
    led_red   = GPIO.PWM(PIN_LED[0], 2000) # Set LED frequencies to 2KHz.
    led_green = GPIO.PWM(PIN_LED[1], 2000)
    buzzer.start(0)
    led_red.start(0)
    led_green.start(0)

    # 3. Ubidots.
    api           = ApiClient(token='A1E-priJupwmfAGtVeOjcslK9wAW16HJzO')
    mqtt_distance = api.get_variable('59e93a20c03f9748c6bc3d54')
    mqtt_speed    = api.get_variable('5a092816c03f9706c0205e88')

#
# tear_down
# Tear down PiCar and GPIO setup.
#
def tear_down():
    # Stop the motor and turn the servo straight.
    bw.stop()
    fw.turn_straight()
    
    # Turn off buzzer and LED.
    buzzer.stop()
    GPIO.output(PIN_BUZZER, GPIO.HIGH)
    led_green.stop()
    led_red.stop()
    GPIO.output(PIN_LED, GPIO.LOW)
    GPIO.cleanup()

#
# calculate_speed
# Calculate speed in cm/s based on motor speed unit (0 ~ 100).
# NOTE: For experimental use only.
#
def calculate_speed(speed_unit):
    return ((speed_unit * 2) / 60) * 6.5 * 3.14

#
# get_opposite_angle
# Returns the opposite turning angle.
#
def get_opposite_angle():
    global last_angle
    if last_angle < 90:
        angle = last_angle + 2 * fw.turning_max
    else:
        angle = last_angle - 2 * fw.turning_max
    last_angle = angle
    return angle

#
# get_random_direction
# Returns a random direction. Depends on FORCE_TURNING flag:
#   0 = random direction
#   1 = force left
#   2 = force right
#   3 = orderdly
#
def get_random_direction():
    global last_angle, last_dir
    if FORCE_TURNING == 0:
        # Randomize direction.
        _dir = random.randint(0, 1)
    elif FORCE_TURNING == 3:
        # Orderly turning.
        _dir = not last_dir
        last_dir = _dir
        #log('Last turn direction: %s' % last_dir)
    else:
        _dir = FORCE_TURNING - 1
    angle = (90 - fw.turning_max) + (_dir * 2 * fw.turning_max)
    last_angle = angle
    return angle

#
# distance_to_stop
# Returns the distance needed to stop the PiCar based on the speed to prevent collision.
#
def distance_to_stop(speed):
    # Return stopping distance based on speed range.
    stopping_distance = DISTANCE_STOP
    if speed > 0 and speed <= 40:
        stopping_distance = 10
    elif speed > 70 and speed <= 100:
        stopping_distance = 30
    return stopping_distance

#
# distance_to_turn
# Returns the distance needed to turn the PiCar based on the speed to avoid collision.
#
def distance_to_turn(speed):
    # Return turning distance based on speed range.
    turning_distance = DISTANCE_TURN
    if speed > 0 and speed <= 40:
        turning_distance = 20
    elif speed > 70 and speed <= 100:
        turning_distance = 40
    return turning_distance

#
# hardware_alert
# Show hardware alerts using the buzzer and LED.
#
def hardware_alert(turn_on, beep_on, color):
    global buzzer, led_green, led_red
    # Turn off.
    if turn_on == False:
        buzzer.ChangeDutyCycle(0.0)
        led_green.ChangeDutyCycle(0.0)
        led_red.ChangeDutyCycle(0.0)
    # Turn on.
    elif turn_on == True:
        # Determine LED duty cycles.
        duty_cycle_R = 0.0
        duty_cycle_G = 0.0
        if color == COLOR_RED:
            duty_cycle_R = 100.0
        elif color == COLOR_GREEN:
            duty_cycle_G = 100.0
        elif color == COLOR_ORANGE:
            duty_cycle_R = 5.0
            duty_cycle_G = 95.0

        # Start LEDs and buzzer.
        led_green.ChangeDutyCycle(duty_cycle_G)
        led_red.ChangeDutyCycle(duty_cycle_R)
        if beep_on == True:
            buzzer.ChangeDutyCycle(50) # 50% duty cycle.

#
# log
# Prints the passed log message with timestamp.
#
def log(log_string):
    print '[{:%Y-%m-%d %H:%M:%S}] %s'.format(datetime.datetime.now()) % log_string

#
# mqtt_collision_avoided
# Send MQTT alert for collision avoidance.
#
def mqtt_collision_avoided(distance, speed):
    global api
    # Send MQTT alert.
    try:
        api.save_collection([
                             {'variable': '59e93a20c03f9748c6bc3d54', 'value': distance, 'context':{'message': 'Collision Avoided'}},
                             {'variable': '5a092816c03f9706c0205e88', 'value': speed,    'context':{'message': 'Collision Avoided'}}
                             ])
    except:
        pass # Ignore exceptions.

#
# mqtt_collision_prevented
# Send MQTT alert for collision prevention.
#
def mqtt_collision_prevented(distance, speed):
    global api
    # Send MQTT alert.
    try:
        api.save_collection([
                             {'variable': '59e93a20c03f9748c6bc3d54', 'value': distance, 'context':{'message': 'Collision Prevented'}},
                             {'variable': '5a092816c03f9706c0205e88', 'value': speed,    'context':{'message': 'Collision Prevented'}}
                             ])
    except:
        pass # Ignore exceptions.

#
# mqtt_speed_limit_warning
# Send MQTT alert if the speed exceeded the limit.
#
def mqtt_speed_limit_warning(speed):
    global mqtt_speed
    # Send MQTT alert.
    try:
        response = mqtt_speed.save_value({'value': speed, 'context':{'message': 'Speed Exceeded Limit of %d' % (speed_limit)}})
    except:
        pass # Ignore exceptions.

#
# mqtt_speed_normal
# Send MQTT alert for normal speed.
#
def mqtt_speed_normal(speed):
    global mqtt_speed
    # Send MQTT alert.
    try:
        response = mqtt_speed.save_value({'value': speed, 'context':{'message': 'Normal Speed of %d' % (speed)}})
    except:
        pass # Ignore exceptions.

#
# start_avoidance
#
def start_avoidance():
    global enable_mqtt
    global speed_fw
    global speed_limit
    over_limit  = False
    last_speed  = 0
    retry_count = 0

    # Read speed limit and start speed from command-line.
    if len(sys.argv) >= 2:
        speed_limit = int(sys.argv[1])
    if len(sys.argv) >= 3:
        speed_fw = int(sys.argv[2])
    if len(sys.argv) >= 4 and argv[3] == "mqtt-off":
        enable_mqtt = False
        log('MQTT disabled.')

    log('* Collision detection and avoidance started *')
    log('Speed limit set to %d.' % speed_limit)

    while True:
        #
        # Task 1: Get distance from the obstacle.
        #         Prevent or avoid collision as necessary.
        #
        distance = ua.get_distance()
        if distance > 0:
            retry_count = 0
            #
            # Level 2. Obstacle detected, but cannot be avoided.
            #
            if distance < distance_to_stop(speed_fw):
                # 1. Send alerts.
                log('Collision prevented: distance = %d cm, speed = %d.' % (distance, speed_fw))
                hardware_alert(True, True, COLOR_RED) # Enable alerts.
                if enable_mqtt == True:
                    thread_cp = Thread(target = mqtt_collision_prevented, args = (distance, speed_fw))
                    thread_cp.start()

                # 2. Turn and backup.
                fw.turn(get_opposite_angle())
                bw.backward()
                bw.speed = SPEED_BW
                time.sleep(1)

                # 3. Now turn, and move forward at random speed.
                angle = get_opposite_angle()
                speed_fw = random.randint(SPEED_FW_MIN, SPEED_FW_MAX)
                log('Backup, turn at %d deg, and go forward at speed %d.' % (angle, speed_fw))
                fw.turn(angle)
                bw.forward()
                bw.speed = speed_fw
                time.sleep(1)
                hardware_alert(False, False, COLOR_NONE) # Disable alerts.
            #
            # Level 1: Obstacle detected, but can be avoided.
            #
            elif distance < distance_to_turn(speed_fw):
                # 1. Send alerts.
                log('Collision avoided: distance = %d cm, speed = %d.' % (distance, speed_fw))
                hardware_alert(True, False, COLOR_GREEN) # Enable alerts.
                if enable_mqtt == True:
                    thread_ca = Thread(target = mqtt_collision_avoided, args = (distance, speed_fw))
                    thread_ca.start()
                
                # 2. Turn away from the obstacle.
                angle = get_random_direction()
                log('Turn at %d deg, and go forward at speed %d.' % (angle, speed_fw))
                fw.turn(angle)
                bw.forward()
                bw.speed = speed_fw
                time.sleep(1)
                hardware_alert(False, False, COLOR_NONE) # Disable alerts.
            #
            # No collision detected.
            #
            else:
                # Keep it steady.
                if speed_fw != last_speed:
                    log('Steady at speed %d.' % speed_fw)
                    if enable_mqtt == True:
                        thread_speed = Thread(target = mqtt_speed_normal, args = (speed_fw, ))
                        thread_speed.start()
                fw.turn_straight()
                bw.forward()
                bw.speed = speed_fw
                last_speed = speed_fw
        # Distance <= 0 for some reason.
        else:
            fw.turn_straight()
            if retry_count > MAX_TRIES:
                # Timed out - stop.
                bw.stop()
                log('Something went horribly wrong; the car is stopped. Distance = %d cm.' % distance)

            else:
                # Backup and try again.
                bw.backward()
                bw.speed = speed_fw
                log('Something went wrong; trying again (%d). Distance = %d cm.' % (retry_count, distance))
                retry_count += 1
        #
        # Task 2: Send a speed limit warning if speed exceeded the limit.
        #
        if not over_limit and speed_fw > speed_limit:
            # Just crossed the speed limit.
            over_limit = True
            log('Speed limit exceeded: current = %d, max = %d.' % (speed_fw, speed_limit))
            hardware_alert(True, False, COLOR_ORANGE) # Enable alerts.
            if enable_mqtt == True:
                thread_slw = Thread(target = mqtt_speed_limit_warning, args = (speed_fw, ))
                thread_slw.start()
        elif over_limit and speed_fw <= speed_limit:
            # Speed just went down below the limit.
            over_limit = False
            hardware_alert(False, False, COLOR_NONE) # Disable alerts.

if __name__ == '__main__':
    try:
        # Setup.
        setup()
        # Start collision detection and avoidance.
        start_avoidance()
    except KeyboardInterrupt:
        log('* Collision detection and avoidance stopped *')
        # Tear everything down.
        tear_down()
