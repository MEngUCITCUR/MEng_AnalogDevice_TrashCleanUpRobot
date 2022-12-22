# Uses the smbus2 library to send and receive data from a
# Simple Motor Controller G2.
# Works on Linux with either Python 2 or Python 3.
#
# NOTE: The SMC's input mode must be "Serial/USB".
# NOTE: You might nee to change the 'SMBus(3)' line below to specify the
#   correct I2C bus device.
# NOTE: You might need to change the 'address = 13' line below to match
#   the device number of your Simple Motor Controller.
from __future__ import division
import time
import serial
# Import the PCA9685 module.
# These modules have been downloaded in the raspberry pi.
import Adafruit_PCA9685
import RPi.GPIO as GPIO
from smbus2 import SMBus, i2c_msg

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

high_speed = 4000  # Max speed
mid_speed = 3200  # Middle speed
low_speed = 800  # low speed

short_delay = 1 # short time period to stop robot and pick up trash
long_delay = 1.5 # long time to stop robot
short_run = 1 # short time for robot to turn left & right
long_run = 2 # long time for robot to turn left & right & move forward
#short_back = 1
long_back = 3 # long time for robot to move backward
#brush_work = 5


steer_servo = 15 #  steer servo connect to PWM 15
servo_pin = 15 #  servo connect to PWM 15

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
GPIO_ECHO_2    = 12

GPIO_TRIGGER_3 = 20
GPIO_ECHO_3    = 21

GPIO_TRIGGER_L =19
GPIO_ECHO_L    =26

GPIO_TRIGGER_R =5
GPIO_ECHO_R    =6

# Define ultrasonic sensors as output
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

class SmcG2I2C(object):
  def __init__(self, bus, address):
    self.bus = bus
    self.address = address

  # Sends the Exit Safe Start command, which is required to drive the motor.
  def exit_safe_start(self):
    write = i2c_msg.write(self.address, [0x83])
    self.bus.i2c_rdwr(write)

  # Sets the SMC's target speed (-3200 to 3200).
  def set_target_speed(self, speed):
    cmd = 0x85  # Motor forward
    if speed < 0:
      cmd = 0x86  # Motor reverse
      speed = -speed
    buffer = [cmd, speed & 0x1F, speed >> 5 & 0x7F]
    write = i2c_msg.write(self.address, buffer)
    self.bus.i2c_rdwr(write)

  # Gets the specified variable as an unsigned value.
  def get_variable(self, id):
    write = i2c_msg.write(self.address, [0xA1, id])
    read = i2c_msg.read(self.address, 2)
    self.bus.i2c_rdwr(write, read)
    b = list(read)
    return b[0] + 256 * b[1]

  # Gets the specified variable as a signed value.
  def get_variable_signed(self, id):
    value = self.get_variable(id)
    if value >= 0x8000:
      value -= 0x10000
    return value

  # Gets the target speed (-3200 to 3200).
  def get_target_speed(self):
    return self.get_variable_signed(20)

  # Gets a number where each bit represents a different error, and the
  # bit is 1 if the error is currently active.
  # See the user's guide for definitions of the different error bits.
  def get_error_status(self):
    return self.get_variable(0)

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

# Open a handle to "/dev/i2c-3", representing the I2C bus.
bus = SMBus(1)

# Select the I2C address of the Simple Motor Controller (the device number).
address_l = 14
address_r = 13

# Add bus and address
smc_l = SmcG2I2C(bus, address_l)
smc_r = SmcG2I2C(bus, address_r)

# start up
smc_l.exit_safe_start()
smc_r.exit_safe_start()

# Check for errors
error_status_l = smc_l.get_error_status()
print("Error status: 0x{:04X}".format(error_status_l))
error_status_r = smc_r.get_error_status()
print("Error status: 0x{:04X}".format(error_status_r))


# target_speed = smc.get_target_speed()
# print("Target speed is {}.".format(target_speed))

# move forward
def forward(speed):
  smc_l.set_target_speed(speed)
  smc_r.set_target_speed(-speed)

# move backward
def backward(speed):
  smc_l.set_target_speed(-speed)
  smc_r.set_target_speed(speed)

# stop the robot
def stopcar():
  smc_l.set_target_speed(0)
  smc_r.set_target_speed(0)

# turn left
def left_turn(speed_left,speed_right):
  smc_l.set_target_speed(speed_left)
  smc_r.set_target_speed(-speed_right)

# turn right
def right_turn(speed_left,speed_right):
  smc_l.set_target_speed(speed_left)
  smc_r.set_target_speed(-speed_right)

# obstacle avoidance system
def obst_avoid():
    print("Obstacle Avoidance")
    sensorval1 = ultra()
    if  sensorval1=="000":
        print(sensorval1+"go forward")
        forward(low_speed) #go forward
        time.sleep(long_run)

    while sensorval1!="000":

        if  sensorval1=="100":
            # stopcar()
            # time.sleep(long_delay)
            # backward(low_speed)
            # time.sleep(short_back)
            stopcar()
            time.sleep(long_delay)
            print(sensorval1+"turn right")
            right_turn(mid_speed,0) # right turn
            time.sleep(long_run)
            sensorval1 = ultra()

        elif  sensorval1=="001":
            # stopcar()
            # time.sleep(long_delay)
            # backward(low_speed)
            # time.sleep(short_back)
            stopcar()
            time.sleep(long_delay)
            print(sensorval1+"turn left")
            left_turn(0,mid_speed) # right turn
            time.sleep(long_run)
            sensorval1 = ultra()

        elif  sensorval1=="110":
            # stopcar()
            # time.sleep(long_delay)
            # backward(low_speed)
            # time.sleep(short_back)
            stopcar()
            time.sleep(long_delay)
            print(sensorval1+"turn right")
            right_turn(mid_speed,0) # right turn
            time.sleep(long_run)
            sensorval1 = ultra()

        elif  sensorval1=="011" :
            # stopcar()
            # time.sleep(long_delay)
            # backward(low_speed)
            # time.sleep(short_back)
            stopcar()
            time.sleep(long_delay)
            print(sensorval1+"turn left")
            left_turn(0,mid_speed) # left turn
            time.sleep(long_run)
            sensorval1 = ultra()

        elif   sensorval1=="111"  or  sensorval1=="101"  or  sensorval1=="010":
            turn = ultra_LnR()
            #turn = "R"
            if turn =="R":
                # stopcar()
                # time.sleep(long_delay)
                # backward(low_speed)
                # time.sleep(short_back)
                stopcar()
                time.sleep(long_delay)
                print(sensorval1+"turn right")
                right_turn(mid_speed,0) # right turn
                time.sleep(long_run)
                sensorval1 = ultra()
            elif turn =="L":
                # stopcar()
                # time.sleep(long_delay)
                # backward(low_speed)
                # time.sleep(short_back)
                stopcar()
                time.sleep(long_delay)
                print(sensorval1+"turn left")
                left_turn(0,mid_speed) # right turn
                time.sleep(long_run)
                sensorval1 = ultra()

        elif   sensorval1=="002":
            #time.sleep(long_delay)
            stopcar()
            time.sleep(long_delay)
            print(sensorval1+"go back and turn sharp left")
            #steer(CENTER)
            backward(low_speed)
            time.sleep(long_back)
            stopcar()
            time.sleep(long_delay)
            #steer(sharp_LEFT)
            left_turn(0,mid_speed) # left turn
            time.sleep(short_run)
            sensorval1 = ultra()
            #steer(CENTER)

        elif   sensorval1=="200":
            #time.sleep(long_delay)
            stopcar()
            time.sleep(long_delay)
            print(sensorval1+"go back and turn sharp right")
            #steer(CENTER)
            backward(low_speed)
            time.sleep(long_back)
            stopcar()
            time.sleep(long_delay)
            #steer(sharp_RIGHT)
            right_turn(mid_speed,0) # right turn
            time.sleep(short_run)
            sensorval1 = ultra()
            #steer(CENTER)

        elif   sensorval1=="210":
            #time.sleep(long_delay)
            stopcar()
            time.sleep(long_delay)
            print(sensorval1+"go back and turn right")
            #steer(CENTER)
            backward(low_speed)
            time.sleep(long_back)
            stopcar()
            time.sleep(long_delay)
            #steer(RIGHT)
            right_turn(mid_speed,0) # right turn
            time.sleep(short_run)
            sensorval1 = ultra()
            #steer(CENTER)

        elif   sensorval1=="012":
            #time.sleep(long_delay)
            stopcar()
            time.sleep(long_delay)
            print(sensorval1+"go back and turn left")
            #steer(CENTER)
            backward(low_speed)
            time.sleep(long_back)
            stopcar()
            time.sleep(long_delay)
            #steer(LEFT)
            left_turn(0,mid_speed) # right turn
            time.sleep(short_run)
            sensorval1 = ultra()
            #steer(CENTER)

        elif   sensorval1=="120" or sensorval1=="201" or sensorval1=="220" or sensorval1=="221" or sensorval1=="211":
            #time.sleep(long_delay)
            stopcar()
            time.sleep(long_delay)
            print(sensorval1+"go back and turn right")
            #steer(CENTER)
            backward(low_speed)
            time.sleep(long_back)
            stopcar()
            time.sleep(long_delay)
            #steer(RIGHT)
            right_turn(mid_speed,0) # right turn
            time.sleep(short_run)
            sensorval1 = ultra()
            #steer(CENTER)

        elif   sensorval1=="020" or sensorval1=="121" or sensorval1=="202" or sensorval1=="212" or sensorval1=="222":
            #time.sleep(long_delay)
            turn = ultra_LnR()
            #turn = "R"
            if turn =="R":
                stopcar()
                time.sleep(long_delay)
                print(sensorval1+"go back and turn right")
                #steer(CENTER)
                backward(low_speed)
                time.sleep(long_back)
                stopcar()
                time.sleep(long_delay)
                #steer(RIGHT)
                right_turn(mid_speed,0) # right turn
                time.sleep(short_run)
                sensorval1 = ultra()
            if turn =="L":
                stopcar()
                time.sleep(long_delay)
                print(sensorval1+"go back and turn left")
                #steer(CENTER)
                backward(low_speed)
                time.sleep(long_back)
                stopcar()
                time.sleep(long_delay)
                #steer(LEFT)
                left_turn(0,mid_speed) # right turn
                time.sleep(short_run)
                sensorval1 = ultra()
                #steer(CENTER)

        elif   sensorval1=="021" or sensorval1=="022" or sensorval1=="122" or sensorval1=="102" or sensorval1=="112":
            #time.sleep(long_delay)
            stopcar()
            time.sleep(long_delay)
            print(sensorval1+"go back and turn left")
            #steer(CENTER)
            backward(low_speed)
            time.sleep(long_back)
            stopcar()
            time.sleep(long_delay)
            #steer(LEFT)
            left_turn(0,mid_speed) # right turn
            time.sleep(short_run)
            sensorval1 = ultra()
            #steer(CENTER)

# receive trash detection information from featherboard
def readfromfeather():
    ser = serial.Serial('/dev/ttyACM1',115200,timeout = 5)
    data = ser.readline()
    label = data.decode('utf-8')
    label = int(label)

    ser.flushInput()
    return label

# change the speed of the brush
def changespeed(left_brush,right_brush):
	pwm.set_pwm(ENA, 0, right_brush)
	pwm.set_pwm(ENB, 0, left_brush)

# start brush to pick up the trash
def trash_pick(left_brush,right_brush):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    changespeed(left_brush,right_brush)

# stop the brush
def stop_clean():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    changespeed(0,0)

# receive trash information and pick up trash
def trash_detect():
    # Trash detection and pick up
    print("Trash detection and pick up")
    #forward(low_speed,low_speed)
    label = readfromfeather()
    print(label)
    if label > 0:
        stopcar()
        time.sleep(short_delay)
        trash_pick(mid_speed,mid_speed)
        time.sleep(short_delay)
        forward(low_speed)
        stopcar()
        stop_clean()
        time.sleep(long_delay)

# boundary distance of the front three ultrasonic sensor.
# distance of robot and obstacle > ob_range_*_long: robot will return 0.
# ob_range_*_long > distance of robot and obstacle > ob_range_*_short: robot will return 1.
# distance of robot and obstacle < ob_range_*_short: robot will return 2.

ob_range_mid_long= 160
ob_range_left_long = 160
ob_range_right_long = 160


ob_range_mid_short= 80
ob_range_left_short = 80
ob_range_right_short = 80

# main function(obstacle avoidance and trash pick up)
try:
    while True :
        obst_avoid()
        trash_detect()





except KeyboardInterrupt:
  # User pressed CTRL-C
  # Reset GPIO settings
  pwm.set_pwm(15, 0, 0)
  GPIO.cleanup()

# new_speed = 3200 if target_speed <= 0 else -3200
# print("Setting target speed to {}.\n".format(new_speed));
# smc.set_target_speed(new_speed)
