from controller import Robot

TIME_STEP = 64
robot = Robot()

wheels = []
wheelsNames = ['left wheel motor', 'right wheel motor']
for i in range(2):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
while robot.step(TIME_STEP) != -1:
    leftSpeed = 2.0
    rightSpeed = 2.0
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    
    