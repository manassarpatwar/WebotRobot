from controller import Robot, Keyboard
import math
import numpy as np

TIME_STEP = 64
robot = Robot()

camera = robot.getCamera("camera")
camera.enable(TIME_STEP)

keyboard = Keyboard()
keyboard.enable(10)

MAX_SPEED = 50.0
MAX_TURN_SPEED = 50.0

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
        
        self.motor_limit = self.leftWheel.motor.getMaxVelocity()

        self.lidar = self.robot.getLidar("lidar")
        self.lidar.enable(TIME_STEP)
        self.distances = []
        self.turning = False
        
        self.error_prior = 0.0
        self.integral = 0.0
        
        self.KP = 400
        self.KI = 2
        self.KD = 500
        
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
        
        threshold = 0.22
        error = threshold-left_sensor
        self.integral += error/TIME_STEP
        derivative = (error - self.error_prior)/TIME_STEP
        output = self.KP*error + self.KI*self.integral + self.KD*derivative
        
        self.error_prior = error
        front_sensor = np.min(self.distances[60:120])
        factor = np.interp(front_sensor, [threshold, 0.5], [0.5,1])

        if not self.turning:
            if front_sensor > threshold:
                self.leftWheel.motor.setVelocity(np.clip(MAX_SPEED*factor+output, a_min = -self.motor_limit, a_max = self.motor_limit))
                self.rightWheel.motor.setVelocity(np.clip(MAX_SPEED*factor-output, a_min = -self.motor_limit, a_max = self.motor_limit))
            else:
                self.stop()
                self.turning = True
        else:
            while front_sensor < threshold:
                self.getDistances()
                front_sensor = np.min(self.distances[60:120])
                factor = 1-np.interp(front_sensor, [threshold, 0.5], [0,1])
                self.leftWheel.motor.setVelocity(MAX_TURN_SPEED*factor)
                self.rightWheel.motor.setVelocity(-MAX_TURN_SPEED*factor)
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
   