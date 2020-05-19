from controller import Robot, Keyboard
import math
from datetime import datetime,timedelta
import numpy as np

TIME_STEP = 64
robot = Robot()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

camera = robot.getCamera("camera")
camera.enable(TIME_STEP)
print(camera)

MAX_SPEED = 15
MAX_TURN_SPEED = 30
ROBOT_ANGULAR_SPEED_IN_DEGREES = 339

class Wheel:
    def __init__(self, name, robot):
        self.position_sensor = robot.getPositionSensor(name)
        self.motor = self.position_sensor.getMotor()
        self.motor.setPosition(float('inf'))
        self.motor.setVelocity(0.0)
        self.target = 0.0
            

class Firefly:
    def __init__(self, robot):
        self.robot = robot
        self.leftWheel = Wheel('left wheel sensor', self.robot)
        self.rightWheel = Wheel('right wheel sensor', self.robot)
        self.prevAngle = None
        self.lidar = self.robot.getLidar("lidar")
        self.lidar.enable(TIME_STEP)
        self.distances = []
    
    def getDistances(self):
         self.distances = np.array(self.lidar.getRangeImage())[[0,89,179]]
    
    def go(self, right, left):
        self.turning = True
        self.leftWheel.motor.setVelocity(left*MAX_SPEED)
        self.rightWheel.motor.setVelocity(right*MAX_SPEED)
        
    def motor_rotate_in_degrees(self, degrees, left, right):
        self.turning = True
        self.leftWheel.motor.setVelocity(left*MAX_TURN_SPEED)
        self.rightWheel.motor.setVelocity(right*MAX_TURN_SPEED)
        duration = abs(degrees) / ROBOT_ANGULAR_SPEED_IN_DEGREES
        start_time = datetime.now()
        while (datetime.now() < start_time + timedelta(seconds=duration)):
            self.robot.step(TIME_STEP)
        self.leftWheel.motor.setVelocity(0)
        self.rightWheel.motor.setVelocity(0)

    def go_for(self, left, right,sec):
        self.turning = True
        self.leftWheel.motor.setVelocity(left*MAX_SPEED)
        self.rightWheel.motor.setVelocity(right*MAX_SPEED)
        start_time = datetime.now()
        while (datetime.now() < start_time + timedelta(seconds=sec)):
            self.robot.step(TIME_STEP)
        self.leftWheel.motor.setVelocity(0)
        self.rightWheel.motor.setVelocity(0)
        


firefly = Firefly(robot)
while firefly.robot.step(TIME_STEP) != -1:
    firefly.getDistances()
    if(firefly.distances[1] > 0.25):
        if (firefly.distances[2] >= 0.20 and firefly.distances[2] <= 0.22):
            firefly.go(1,1)
        elif firefly.distances[2] > 0.22:
            firefly.motor_rotate_in_degrees(15,1,-1)
            firefly.go_for(1,1,0.02)
            firefly.motor_rotate_in_degrees(15,-1,1)
        else: 
            firefly.motor_rotate_in_degrees(10,-1,1)
            firefly.go_for(1,1,0.02)
            firefly.motor_rotate_in_degrees(10,1,-1)
    else:
        firefly.motor_rotate_in_degrees(10,-1,1)
   
   
    
    