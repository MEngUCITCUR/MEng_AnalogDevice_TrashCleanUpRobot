from __future__ import division
import time
# Import the PCA9685 module.
# These modules have been downloaded in the raspberry pi.
import Adafruit_PCA9685
import RPi.GPIO as GPIO
# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()
high_speed = 4000  # Max speed
mid_speed = 1200  # Middle speed
low_speed = 1000  # low speed

#short_delay = 1
#long_delay = 3
short_run = 1 # move forward for a short time
long_run = 1.5 # move forward for a long time
short_back = 1 # time for move back
#long_back = 1.5
#brush_work = 5

steer_servo = 15 #  steer servo connect to PWM 15
servo_pin = 15 #  servo connect to PWM 15

CENTER= 350 #Steer servo car go forward
RIGHT = 400 #Steer servo car turn right
LEFT = 300 #Steer servo car turn left
sharp_LEFT = 325 #Steer servo car turn sharp left
sharp_RIGHT = 375 #Steer servo car turn sharp right

# boundary distance of the front three ultrasonic sensor.
# distance of robot and obstacle > ob_range_*_long: robot will return 0.
# ob_range_*_long > distance of robot and obstacle > ob_range_*_short: robot will return 1.
# distance of robot and obstacle < ob_range_*_short: robot will return 2.

ob_range_mid_long= 60
ob_range_left_long = 60
ob_range_right_long = 60


ob_range_mid_short= 30
ob_range_left_short = 30
ob_range_right_short = 30

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
GPIO.setmode(GPIO.BCM) # GPIO number  in BCM mode
GPIO.setwarnings(False)
#define L298N(Model-Pi motor drive board) GPIO pins
IN1 = 23  #right motor direction pin
IN2 = 24  #right motor direction pin
IN3 = 27  #left motor direction pin
IN4 = 22  #left motor direction pin
ENA = 0  #Right motor speed PCA9685 port 0
ENB = 1  #Left motor speed PCA9685 port 1

# Define GPIO to use on Pi
GPIO_TRIGGER_1 = 15
GPIO_ECHO_1    = 14

GPIO_TRIGGER_2 = 16
GPIO_ECHO_2    = 25

GPIO_TRIGGER_3 = 20
GPIO_ECHO_3    = 21

GPIO_TRIGGER_L =19
GPIO_ECHO_L    =26

GPIO_TRIGGER_R =5
GPIO_ECHO_R    =6

# Define motor control  pins as output
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(GPIO_TRIGGER_1, GPIO.OUT)
GPIO.setup(GPIO_ECHO_1, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_2, GPIO.OUT)
GPIO.setup(GPIO_ECHO_2, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_3, GPIO.OUT)
GPIO.setup(GPIO_ECHO_3, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_L, GPIO.OUT)
GPIO.setup(GPIO_ECHO_L, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_R, GPIO.OUT)
GPIO.setup(GPIO_ECHO_R, GPIO.IN)

# left of the front three ultrasonic sensors detect distance
def measure_1():
  # This function measures a distance
  GPIO.output(GPIO_TRIGGER_1, True)
  time.sleep(0.00001)
  GPIO.output(GPIO_TRIGGER_1, False)
  start = time.time()
  while GPIO.input(GPIO_ECHO_1)==0:
    start = time.time()
  while GPIO.input(GPIO_ECHO_1)==1:
    stop = time.time()
  elapsed = stop-start
  distance1 = (elapsed * 34300)/2
  return distance1

# mid of the front three ultrasonic sensors detect distance
def measure_2():
  # This function measures a distance
  GPIO.output(GPIO_TRIGGER_2, True)
  time.sleep(0.00001)
  GPIO.output(GPIO_TRIGGER_2, False)
  start = time.time()
  while GPIO.input(GPIO_ECHO_2)==0:
    start = time.time()
  while GPIO.input(GPIO_ECHO_2)==1:
    stop = time.time()
  elapsed = stop-start
  distance2 = (elapsed * 34300)/2
  return distance2

# right of the front three ultrasonic sensors detect distance
def measure_3():
  # This function measures a distance
  GPIO.output(GPIO_TRIGGER_3, True)
  time.sleep(0.00001)
  GPIO.output(GPIO_TRIGGER_3, False)
  start = time.time()
  while GPIO.input(GPIO_ECHO_3)==0:
    start = time.time()
  while GPIO.input(GPIO_ECHO_3)==1:
    stop = time.time()
  elapsed = stop-start
  distance3 = (elapsed * 34300)/2
  return distance3

# ultrasonic sensor on the left side of the robot detect distance
def measure_L():
  # This function measures a distance
  GPIO.output(GPIO_TRIGGER_L, True)
  time.sleep(0.00001)
  GPIO.output(GPIO_TRIGGER_L, False)
  start = time.time()
  while GPIO.input(GPIO_ECHO_L)==0:
    start = time.time()
  while GPIO.input(GPIO_ECHO_L)==1:
    stop = time.time()
  elapsed = stop-start
  distanceL = (elapsed * 34300)/2
  return distanceL

# ultrasonic sensor on the right side of the robot detect distance
def measure_R():
  # This function measures a distance
  GPIO.output(GPIO_TRIGGER_R, True)
  time.sleep(0.00001)
  GPIO.output(GPIO_TRIGGER_R, False)
  start = time.time()
  while GPIO.input(GPIO_ECHO_R)==0:
    start = time.time()
  while GPIO.input(GPIO_ECHO_R)==1:
    stop = time.time()
  elapsed = stop-start
  distanceR = (elapsed * 34300)/2
  return distanceR

# change speed function
def changespeed(speed_left,speed_right):
	pwm.set_pwm(ENA, 0, speed_right)
	pwm.set_pwm(ENB, 0, speed_left)

# stop the robot
def stopcar():
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.LOW)
	GPIO.output(IN4, GPIO.LOW)
	changespeed(0,0)

# move forward
def forward(speed_left,speed_right):
	GPIO.output(IN1, GPIO.HIGH)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.HIGH)
	GPIO.output(IN4, GPIO.LOW)
	changespeed(speed_left,speed_right)

# move backward
def backward(speed_left,speed_right):
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN2, GPIO.HIGH)
	GPIO.output(IN3, GPIO.LOW)
	GPIO.output(IN4, GPIO.HIGH)
	changespeed(speed_left,speed_right)

# turn left
def left_turn(speed_left,speed_right):
	GPIO.output(IN1, GPIO.HIGH)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.LOW)
	GPIO.output(IN4, GPIO.HIGH)
	changespeed(speed_left,speed_right)

# turn right
def right_turn(speed_left,speed_right):
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN2, GPIO.HIGH)
	GPIO.output(IN3, GPIO.HIGH)
	GPIO.output(IN4, GPIO.LOW)
	changespeed(speed_left,speed_right)

# the function to control the steering system in the front of the robot
def steer(angle):
	if angle>RIGHT :
		angle=RIGHT
	if angle<LEFT :
		angle=LEFT
	pwm.set_pwm(steer_servo, 0, angle)

# judgement function for the front three ultrasonic sensor
# distance of robot and obstacle > ob_range_*_long: robot will return 0.
# ob_range_*_long > distance of robot and obstacle > ob_range_*_short: robot will return 1.
# distance of robot and obstacle < ob_range_*_short: robot will return 2.
# it will return a 3-bit digit
def ultra():
    print("Ultrasonic forward detect")
    distance1 = measure_1()
    if distance1>ob_range_left_long:
        sts1 =  0
    elif distance1 >ob_range_left_short and distance1 < ob_range_left_long:
        sts1 =  1
    elif distance1 <ob_range_left_short:
        sts1 =  2
    distance2 = measure_2()
    if distance2>ob_range_mid_long:
        sts2 =  0
    elif distance2 >ob_range_mid_short and distance2 < ob_range_mid_long:
        sts2 =  1
    elif distance2 <ob_range_mid_short:
        sts2 =  2
    distance3 = measure_3()
    if distance3>ob_range_right_long:
        sts3 =  0
    elif distance3 >ob_range_right_short and distance3 < ob_range_right_long:
        sts3 =  1
    elif distance3 <ob_range_right_short:
        sts3 =  2

    sensorval1 = ''.join([str(sts1), str(sts2), str(sts3)])
    return sensorval1

# judgement function for the left side and right side ultrasonic sensor
# distance of robot and left side obstacle > distance of robot and left side obstacle: turn left
# distance of robot and left side obstacle <= distance of robot and left side obstacle: turn right

def ultra_LnR():
    print("Ultrasonic left and right detect")
    distanceL = measure_L()
    distanceR = measure_R()

    if distanceR >= distanceL:
        turn = "R"
    elif distanceR < distanceL:
        turn = "L"
    print(turn)
    return turn

# obstacle avoidance system
def obst_avoid():
    print("Obstacle Avoidance")
    sensorval1 = ultra()
    if   sensorval1=="000":

        print(sensorval1+"go forward")
        steer(CENTER)
        forward(low_speed,low_speed) #go forward
        time.sleep(long_run)

    while sensorval1!="000":

        if  sensorval1=="100":
            #time.sleep(long_delay)
            stopcar()
            print(sensorval1+"turn right")
            steer(RIGHT)
            forward(mid_speed,low_speed) # right turn
            time.sleep(long_run)
            sensorval1 = ultra()
            #steer(CENTER)


        elif  sensorval1=="001":
            #time.sleep(long_delay)
            stopcar()
            print(sensorval1+"turn left")
            steer(LEFT)
            forward(low_speed,mid_speed) # left turn
            time.sleep(long_run)
            sensorval1 = ultra()
            #steer(CENTER)

        elif  sensorval1=="110":
            #time.sleep(long_delay)
            stopcar()
            print(sensorval1+"turn right")
            steer(RIGHT)
            forward(mid_speed,low_speed) # right turn
            time.sleep(long_run)
            sensorval1 = ultra()
            #steer(CENTER)

        elif  sensorval1=="011" :
            #time.sleep(long_delay)
            stopcar()
            print(sensorval1+"turn left")
            steer(LEFT)
            forward(low_speed,mid_speed) # left turn
            time.sleep(long_run)
            sensorval1 = ultra()
            #steer(CENTER)

        elif   sensorval1=="111"  or  sensorval1=="101"  or  sensorval1=="010":
            #time.sleep(long_delay)
            turn = ultra_LnR()
            if turn =="R":
                stopcar()
                print(sensorval1+"turn right")
                steer(RIGHT)
                forward(mid_speed,low_speed) # right turn
                time.sleep(long_run)
                sensorval1 = ultra()
                #steer(CENTER)
            elif turn =="L":
                stopcar()
                print(sensorval1+"turn left")
                steer(LEFT)
                forward(low_speed,mid_speed) # right turn
                time.sleep(long_run)
                sensorval1 = ultra()
                #steer(CENTER)


        elif   sensorval1=="002":
            #time.sleep(long_delay)
            stopcar()
            print(sensorval1+"go back and turn left")
            steer(CENTER)
            backward(low_speed,low_speed)
            time.sleep(short_back)
            stopcar()
            steer(LEFT)
            forward(low_speed,mid_speed) # right turn
            time.sleep(short_run)
            sensorval1 = ultra()
            #steer(CENTER)

        elif   sensorval1=="200":
            #time.sleep(long_delay)
            stopcar()
            print(sensorval1+"go back and turn right")
            steer(CENTER)
            backward(low_speed,low_speed)
            time.sleep(short_back)
            stopcar()
            steer(RIGHT)
            forward(mid_speed,low_speed) # right turn
            time.sleep(short_run)
            sensorval1 = ultra()
            #steer(CENTER)

        elif   sensorval1=="210":
            #time.sleep(long_delay)
            stopcar()
            print(sensorval1+"go back and turn right")
            steer(CENTER)
            backward(low_speed,low_speed)
            time.sleep(short_back)
            stopcar()
            steer(RIGHT)
            forward(mid_speed,low_speed) # right turn
            time.sleep(short_run)
            sensorval1 = ultra()
            #steer(CENTER)

        elif   sensorval1=="012":
            #time.sleep(long_delay)
            stopcar()
            print(sensorval1+"go back and turn left")
            steer(CENTER)
            backward(low_speed,low_speed)
            time.sleep(short_back)
            stopcar()
            steer(LEFT)
            forward(low_speed,mid_speed) # right turn
            time.sleep(short_run)
            sensorval1 = ultra()
            #steer(CENTER)

        elif   sensorval1=="120" or sensorval1=="201" or sensorval1=="220" or sensorval1=="221" or sensorval1=="211":
            #time.sleep(long_delay)
            stopcar()
            print(sensorval1+"go back and turn right")
            steer(CENTER)
            backward(low_speed,low_speed)
            time.sleep(short_back)
            stopcar()
            steer(RIGHT)
            forward(mid_speed,low_speed) # right turn
            time.sleep(short_run)
            sensorval1 = ultra()
            #steer(CENTER)

        elif   sensorval1=="020" or sensorval1=="121" or sensorval1=="202" or sensorval1=="212" or sensorval1=="222":
            #time.sleep(long_delay)
            turn = ultra_LnR()
            if turn =="R":
                stopcar()
                print(sensorval1+"go back and turn right")
                steer(CENTER)
                backward(low_speed,low_speed)
                time.sleep(short_back)
                stopcar()
                steer(RIGHT)
                forward(mid_speed,low_speed) # right turn
                time.sleep(short_run)
                sensorval1 = ultra()
            elif turn =="L":
                stopcar()
                print(sensorval1+"go back and turn left")
                steer(CENTER)
                backward(low_speed,low_speed)
                time.sleep(short_back)
                stopcar()
                steer(LEFT)
                forward(low_speed,mid_speed) # right turn
                time.sleep(short_run)
                sensorval1 = ultra()
                #steer(CENTER)

        elif   sensorval1=="021" or sensorval1=="022" or sensorval1=="122" or sensorval1=="102" or sensorval1=="112":
            #time.sleep(long_delay)
            stopcar()
            print(sensorval1+"go back and turn left")
            steer(CENTER)
            backward(low_speed,low_speed)
            time.sleep(short_back)
            stopcar()
            steer(LEFT)
            forward(low_speed,mid_speed) # right turn
            time.sleep(short_run)
            sensorval1 = ultra()
            #steer(CENTER)

# main function
try:
    while True :
        obst_avoid()

except KeyboardInterrupt:
  # User pressed CTRL-C
  # Reset GPIO settings
  pwm.set_pwm(15, 0, 0)
  GPIO.cleanup()

