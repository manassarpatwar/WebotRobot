from controller import Robot, Keyboard
import math
import numpy as np

TIME_STEP = 64
robot = Robot()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

camera = robot.getCamera("camera")
camera.enable(TIME_STEP)
print(camera)

MAX_SPEED = 100.0
MAX_TURN_SPEED = 30.0

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
        self.moving = False
        self.leftWheel = Wheel('left wheel sensor', self.robot)
        self.rightWheel = Wheel('right wheel sensor', self.robot)
        self.prevAngle = None
        self.turning = False
        self.movingFunc = self.stop
        self.inertialUnit = robot.getInertialUnit('inertial unit')
        self.target = None
        self.lidar = self.robot.getLidar("lidar")
        self.lidar.enable(TIME_STEP)
        self.distances = []
    
    def getDistances(self):
         self.distances = np.array(self.lidar.getRangeImage())[[0,89,179]]
    
    def stop(self):
        self.leftWheel.motor.setVelocity(0.0)
        self.rightWheel.motor.setVelocity(0.0)
        
    def checkTurn(self):
        res = abs(round(self.inertialUnit.getRollPitchYaw()[2],2) - self.target)
        self.turning = res > 0.01
        if(not self.turning):
            self.target = None
            self.movingFunc = self.stop
            self.inertialUnit.disable()
            
        return res
        
    def enableInertialUnit(self, movingFunc):
        self.inertialUnit.enable(TIME_STEP)
        self.movingFunc = movingFunc
    
    def left(self):
        self.turning = True
        if(self.target is None):
            self.target = self.inertialUnit.getRollPitchYaw()[2]+math.pi/2
            self.target = self.target*-1+math.pi if abs(self.target) > math.pi else self.target
        factor = np.interp(abs(self.inertialUnit.getRollPitchYaw()[2] - self.target), [0, math.pi/3], [0.01,1])
        self.leftWheel.motor.setVelocity(-MAX_TURN_SPEED*factor)
        self.rightWheel.motor.setVelocity(MAX_TURN_SPEED*factor)
        
    def right(self):
        self.turning = True
        if(self.target is None):
            self.target = self.inertialUnit.getRollPitchYaw()[2]-math.pi/2
            self.target = self.target*-1-math.pi if abs(self.target) > math.pi else self.target
        factor = np.interp(abs(self.inertialUnit.getRollPitchYaw()[2] - self.target), [0, math.pi/3], [0.01,1])
        self.leftWheel.motor.setVelocity(MAX_TURN_SPEED*factor)
        self.rightWheel.motor.setVelocity(-MAX_TURN_SPEED*factor)
        
    def turnAround(self):
        self.turning = True
        current = self.inertialUnit.getRollPitchYaw()[2]
        if(self.target is None):
            self.target = current+math.pi
            self.target = self.target-2*math.pi if abs(self.target) > math.pi else self.target        
        factor = np.interp(abs(self.inertialUnit.getRollPitchYaw()[2] - self.target), [0, math.pi/3], [0.01,1])
        self.leftWheel.motor.setVelocity(MAX_TURN_SPEED*factor)
        self.rightWheel.motor.setVelocity(-MAX_TURN_SPEED*factor)    
    
    def forward(self):
        factor = np.interp(self.distances[1], [0.25, 0.5], [0.01,1])
        self.leftWheel.motor.setVelocity(MAX_SPEED*factor)
        self.rightWheel.motor.setVelocity(MAX_SPEED*factor)
        


firefly = Firefly(robot)
while firefly.robot.step(TIME_STEP) != -1:
    firefly.getDistances()
    firefly.movingFunc()
    if firefly.turning:
        firefly.checkTurn()
    else:
        if(firefly.distances[1] > 0.25):
            firefly.movingFunc = firefly.forward
        else:
            firefly.stop()
            if(firefly.distances[0] > 0.35):
                firefly.enableInertialUnit(firefly.left)
            elif(firefly.distances[2] > 0.35):
                firefly.enableInertialUnit(firefly.right)
            else:
                firefly.enableInertialUnit(firefly.turnAround)
   
    
    