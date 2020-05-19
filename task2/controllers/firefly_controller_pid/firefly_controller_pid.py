from controller import Robot, Keyboard
import math
import numpy as np

TIME_STEP = 64
robot = Robot()

camera = robot.getCamera("camera")
camera.enable(TIME_STEP)

keyboard = Keyboard()
keyboard.enable(10)

MAX_SPEED = 25.0
MAX_TURN_SPEED = 25.0

class Wheel:
    def __init__(self, name, robot):
        self.position_sensor = robot.getPositionSensor(name)
        self.motor = self.position_sensor.getMotor()
        self.motor.setPosition(float('inf'))
        self.motor.setVelocity(0.0)
            
class Firefly:
    def __init__(self, robot):
        self.robot = robot

        self.leftWheel = Wheel('left wheel sensor', self.robot)
        self.rightWheel = Wheel('right wheel sensor', self.robot)

        self.lidar = self.robot.getLidar("lidar")
        self.lidar.enable(TIME_STEP)
        self.distances = []
        self.turning = False
        
        self.error_prior = 0.0
        self.integral = 0.0
        
        self.KP = 100
        self.KI = 0
        self.KD = 20
        
    def getDistances(self):
        self.distances = np.array(self.lidar.getRangeImage())
            
    def stop(self):
        self.leftWheel.motor.setVelocity(0.0)
        self.rightWheel.motor.setVelocity(0.0)
        
    def move(self):
        left_distances = self.distances[0:60]
        left_min = np.min(left_distances)
        left_sensor = left_distances[np.where(np.logical_and(left_distances >= left_min, left_distances < left_min+0.05))]
        left_sensor = np.mean(left_sensor)
        
        threshold = 0.275
        error = threshold-left_sensor
        self.integral += error
        derivative = (error - self.error_prior)
        output = self.KP*error + self.KI*self.integral + self.KD*derivative
        
        self.error_prior = error
        front_sensor = self.distances[89]
        factor = np.interp(front_sensor, [threshold, 0.5], [0.1,1])

        if not self.turning:
            if front_sensor > threshold:
                self.leftWheel.motor.setVelocity(MAX_SPEED*factor+output)
                self.rightWheel.motor.setVelocity(MAX_SPEED*factor-output)
            else:
                self.stop()
                self.turning = True
        else:
            while front_sensor < threshold:
                self.getDistances()
                front_sensor = self.distances[89]
                self.leftWheel.motor.setVelocity(MAX_TURN_SPEED)
                self.rightWheel.motor.setVelocity(-MAX_TURN_SPEED)
                self.robot.step(TIME_STEP)
            self.turning = False
    

firefly = Firefly(robot)

while firefly.robot.step(TIME_STEP) != -1:
    firefly.getDistances()    
    firefly.move()
    # keys=[keyboard.getKey() for i in range(4)]
        
    # leftSpeed = 0
    # rightSpeed = 0
    # if (Keyboard.UP in keys):
        # leftSpeed = MAX_SPEED
        # rightSpeed = MAX_SPEED
    # if (Keyboard.DOWN in keys):
        # leftSpeed = -MAX_SPEED
        # rightSpeed = -MAX_SPEED
    # if (Keyboard.LEFT in keys):
        # leftSpeed = -MAX_SPEED
        # rightSpeed = MAX_SPEED
    # if (Keyboard.RIGHT in keys):
        # leftSpeed = MAX_SPEED
        # rightSpeed = -MAX_SPEED
        
    # firefly.leftWheel.motor.setVelocity(leftSpeed)
    # firefly.rightWheel.motor.setVelocity(rightSpeed)
   