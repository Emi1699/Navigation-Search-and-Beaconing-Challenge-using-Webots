from controller import Robot, Lidar, Camera, CameraRecognitionObject

TIME_STEP = 64
robot = Robot()

robotSpeed = 6

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
    
def turnRight():
    wheels[0].setVelocity(robotSpeed)
    wheels[1].setVelocity(robotSpeed)
    wheels[2].setVelocity(-robotSpeed)
    wheels[3].setVelocity(-robotSpeed)
    
def turnLeft():
    wheels[0].setVelocity(-robotSpeed)
    wheels[1].setVelocity(-robotSpeed)
    wheels[2].setVelocity(robotSpeed)
    wheels[3].setVelocity(robotSpeed)

def stop():
    for i in range(4):
        wheels[i].setVelocity(0)
        
def moveForward():    
    for i in range(4):
        wheels[i].setVelocity(robotSpeed)
        
def moveBackwards():
    for i in range(4):
        wheels[i].setVelocity(-robotSpeed)
        
def readLidarData():
    lidarData = lidar.getRangeImage()
    return lidarData
    
def avoidObstacles(lidarData):
    lidarLeft = lidarData[(128 - 32):(128 + 32)]
    lidarFront = lidarData[(256 - 14):(256 +  14)]
    lidarFrontRight = lidarData[(320 - 15):(320 + 30)]
    lidarFrontLeft = lidarData[(192 - 30):(192 + 15)]
    lidarRight = lidarData[(384 - 32):(384 + 32)]
    
    stopEverythingElse = 0
    
    
    if max(lidarFront) < 0.5:
        if max(lidarFrontLeft) > 0.65:
            turnLeft()
        elif max(lidarFrontRight) > 0.65:
            turnRight()
        else:
            moveBackwards()
            stopEverythingElse = 1
    elif stopEverythingElse != 1:
        if max(lidarFrontLeft) < 0.425:
            turnRight()
        elif max(lidarFrontRight) < 0.425:
            turnLeft()
        else:
            moveForward()
       
while flag == True:
    if robot.step(TIME_STEP) == -1:
        flag = False
    
    lidarData = readLidarData()
    
    #print lidarData from bottom, left, top, right
    # print("  0: " + str(lidarData[0]))
    # print("128: " + str(lidarData[128]))
    # print("256: " + str(lidarData[256]))
    # print("384: " + str(lidarData[384]))
            
    if start == True:
        colour = getStartColour()
        # print(colour)
        start = False
      
    moveForward()  
    avoidObstacles(lidarData)
        
    # print(foundColour(colour))
    
    # flag = False
        