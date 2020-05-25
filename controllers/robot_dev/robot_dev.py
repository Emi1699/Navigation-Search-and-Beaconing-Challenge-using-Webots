from controller import Robot, Lidar, Camera, CameraRecognitionObject

TIME_STEP = 64
robot = Robot()

wheels = []
wheelsNames = ['wheelleft1', 'wheelleft2', 'wheelright1', 'wheelright2']
for i in range(4):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)


lidar = robot.getLidar("lidar_360")
lidar.enable(TIME_STEP)

camera_front = robot.getCamera("camera_front")
camera_front.enable(TIME_STEP)
camera_front.recognitionEnable(TIME_STEP)

camera_left = robot.getCamera("camera_left")
camera_left.enable(TIME_STEP)
camera_left.recognitionEnable(TIME_STEP)

camera_right = robot.getCamera("camera_right")
camera_right.enable(TIME_STEP)
camera_right.recognitionEnable(TIME_STEP)

flag = True
start = True

def getStartColour():
    if camera_right.getRecognitionNumberOfObjects() != 0 :
        obj = camera_right.getRecognitionObjects()
        colour = obj[0].get_colors()
        return colour
    else :
        return False

def foundColour(c):
    if camera_front.getRecognitionNumberOfObjects() != 0 :
        obj = camera_front.getRecognitionObjects()
        colour = obj[0].get_colors()
        if c == colour:
            return True
        else:
            return False
    return False
       
while flag == True:
    if robot.step(TIME_STEP) == -1:
        flag = False
            
    if start == True:
        colour = getStartColour()
        print(colour)
        start = False
        
    print(foundColour(colour))
    
    flag = False
        