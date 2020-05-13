from controller import Robot, Keyboard


TIME_STEP = 64
robot = Robot()
keyboard = Keyboard()
keyboard.enable(10)

lidar = robot.getLidar("lidar")
lidar.enable(10)

camera = robot.getCamera("camera")
camera.enable(10)
print(camera)

MAX_SPEED = 10.0

wheels = []
wheelsNames = ['left wheel motor', 'right wheel motor']
for i in range(2):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
while robot.step(TIME_STEP) != -1:
    keys=[keyboard.getKey() for i in range(4)]
    print(lidar.getRangeImage()[90])
    
    leftSpeed = 0
    rightSpeed = 0
    if (Keyboard.UP in keys):
        leftSpeed = MAX_SPEED
        rightSpeed = MAX_SPEED
    if (Keyboard.DOWN in keys):
        leftSpeed = -MAX_SPEED
        rightSpeed = -MAX_SPEED
    if (Keyboard.LEFT in keys):
        leftSpeed = -MAX_SPEED
        rightSpeed = MAX_SPEED
    if (Keyboard.RIGHT in keys):
        leftSpeed = MAX_SPEED
        rightSpeed = -MAX_SPEED
        
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
   
    
    