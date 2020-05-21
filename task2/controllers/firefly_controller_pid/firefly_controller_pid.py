from controller import Robot, Keyboard
import math
import numpy as np

TIME_STEP = 64
robot = Robot()

keyboard = Keyboard()
keyboard.enable(10)

MAX_SPEED = 40.0
MAX_TURN_SPEED = 40.0

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
        self.camera = self.robot.getCamera("camera")
        self.camera.enable(TIME_STEP)
        self.camera.recognitionEnable(TIME_STEP)
        self.distances = []
        self.turning = False
        
        self.error_prior = 0.0
        self.integral = 0.0
        
        self.KP = 200
        self.KI = 2
        self.KD = 500
        
        self.threshold = 0.22
        
        self.colors = [[0.5,0,0], 
                        [1,0,0], 
                        [0.5,0.5,0],
                        [1,1,0],
                        [0,0.5,0],
                        [0,1,0],
                        [0,0.5,0.5],
                        [0,1,1],
                        [0,0,0.5],
                        [0,0,1],
                        [0.5,0,0.5],
                        [1,0,1]]
        self.target_color = None
        
        self.done = False
        
        
    def getTargetColor(self):
        while self.target_color is None:
            objs = self.camera.getRecognitionObjects()
            if len(objs) > 0:
                potential_target = objs[0].get_colors()
                if any(potential_target == c for c in self.colors):
                    self.target_color = potential_target
            self.turn()

        self.getDistances()
        front_sensor = np.min(self.distances[60:120])
        while front_sensor < 0.5:
            self.getDistances()
            front_sensor = np.min(self.distances[60:120])
            factor = 1-np.interp(front_sensor, [self.threshold, 0.5], [0,1])
            self.turn(factor)
        self.stop()
    
    def turn(self, factor=1):
            self.leftWheel.motor.setVelocity(MAX_TURN_SPEED*factor)
            self.rightWheel.motor.setVelocity(-MAX_TURN_SPEED*factor)
            self.robot.step(TIME_STEP)
    
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
        
        error = self.threshold-left_sensor
        self.integral += error/TIME_STEP
        derivative = (error - self.error_prior)/TIME_STEP
        output = self.KP*error + self.KI*self.integral + self.KD*derivative
        
        self.error_prior = error
        front_sensor = np.min(self.distances[60:120])
        factor = np.interp(front_sensor, [self.threshold, 0.5], [0.5,1])

        if not self.turning:
            if front_sensor > self.threshold:
                self.leftWheel.motor.setVelocity(np.clip(MAX_SPEED*factor+output, a_min = -self.motor_limit, a_max = self.motor_limit))
                self.rightWheel.motor.setVelocity(np.clip(MAX_SPEED*factor-output, a_min = -self.motor_limit, a_max = self.motor_limit))
            else:
                self.stop()
                objs = self.camera.getRecognitionObjects()
                if len(objs) > 0:
                    obj_colors = [o.get_colors() for o in objs]
                    if any(c == self.target_color for c in obj_colors):
                        self.done = True
                self.turning = True
        else:
            while front_sensor < self.threshold:
                self.getDistances()
                front_sensor = np.min(self.distances[60:120])
                factor = 1-np.interp(front_sensor, [self.threshold, 0.5], [0,1])
                self.turn(factor)
            self.turning = False
    

firefly = Firefly(robot)
firefly.getTargetColor()
# firefly.target_color = [0,0.5,0.5]
print(firefly.target_color)
while firefly.robot.step(TIME_STEP) != -1:
    if not firefly.done:
        firefly.getDistances()    
        firefly.move()
   