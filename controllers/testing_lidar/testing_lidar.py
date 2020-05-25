from controller import Robot, Lidar

TIME_STEP = 64
robot = Robot()

robotSpeed = 6
wallDistanceThreshold = 0.14

wheels = []
wheelsNames = ['wheelleft1', 'wheelleft2', 'wheelright1', 'wheelright2']
for i in range(4):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(robotSpeed)


lidar = robot.getLidar("lidar_360")
lidar.enable(TIME_STEP)

def readLidarData():
    lidarData = lidar.getRangeImage()
    return lidarData


def average(list):
    return sum(list)/len(list)
        
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

              
               
while robot.step(TIME_STEP) != -1:
    #read lidar data
    lidarData = readLidarData()
    
    #select lidar data needed for detecting walls and shit
    lidarFront = lidarData[(256 - 12):(256 +  12)]
    lidarFrontRight = lidarData[(320 - 32):(320 + 32)]
    
    
    print("front: " + str(min(lidarFront)))
    print("right: " + str(max(lidarFrontRight)))
    print(" ")
    
    # > = off
    # < < on
        
    if min(lidarFront) > 0.31 and min(lidarFrontRight) > 0.31:
        turnRight()
    if min(lidarFront) < 0.31 and min(lidarFrontRight) > 0.31:
        turnLeft()
    if min(lidarFront) > 0.31 and min(lidarFrontRight) < 0.31:
        moveForward()
    if min(lidarFront) < 0.31 and min(lidarFrontRight) < 0.31:
        turnLeft()
        
        
        
    
    
    
    
    
    
    
    
    