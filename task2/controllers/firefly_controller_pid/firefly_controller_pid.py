from controller import Robot, Keyboard
import math
import numpy as np

TIME_STEP = 64
robot = Robot()

camera = robot.getCamera("camera")
camera.enable(TIME_STEP)

keyboard = Keyboard()
keyboard.enable(10)

MAX_SPEED = 20.0
MAX_TURN_SPEED = 30.0

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
        
        self.error_prior = 0.0
        self.integral = 0.0
        
        self.KP = 100
        self.KI = 0
        self.KD = 20
        
    def getDistances(self):
        self.distances = np.array(self.lidar.getRangeImage())
            
        
    def move(self):
        error = 0.2-np.sum(self.distances[0:75])/75.0
        self.integral += error
        derivative = (error - self.error_prior)
        output = self.KP*error + self.KI*self.integral + self.KD*derivative
        
        self.error_prior = error
        factor = np.interp(np.sum(self.distances[75:105])/30.0, [0.2, 0.5], [0,1])
        self.leftWheel.motor.setVelocity(MAX_SPEED*factor+output)
        self.rightWheel.motor.setVelocity(MAX_SPEED*factor-output)
    
    

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
   