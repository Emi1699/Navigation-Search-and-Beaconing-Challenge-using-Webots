from controller import Robot, Lidar

TIME_STEP = 64

robot = Robot()

wheels = []
wheelsNames = ['wheelleft1', 'wheelleft2', 'wheelright1', 'wheelright2']
for i in range(4):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)


lidar_front = robot.getLidar("lidar_front")
lidar_front.enable(TIME_STEP)

lidar_left = robot.getLidar("lidar_left")
lidar_left.enable(TIME_STEP)

lidar_right = robot.getLidar("lidar_right")
lidar_right.enable(TIME_STEP)

lidar_back = robot.getLidar("lidar_back")
lidar_back.enable(TIME_STEP)


def obstacleInfront():
    if lidar_front.getRangeImage()[205]<0.2 and lidar_front.getRangeImage()[205] != 0.0:
        return True
    else:
        return False
    
def maxDistance():
    right_max = max(lidar_right.getRangeImage())
    left_max = max(lidar_left.getRangeImage())
    
    if right_max>left_max:
        return True, right_max
    else:
        return False, left_max
        
def turnRight(goal):
    reading = int(1000*(lidar_front.getRangeImage()[205]))
    goal = int(goal*1000)
    if reading == goal:
        for i in range(4):
            wheels[i].setVelocity(0.0)
            return true
    else :
        wheels[0].setVelocity(2.0)
        wheels[1].setVelocity(2.0)
        wheels[2].setVelocity(0.0)
        wheels[3].setVelocity(0.0)    

        
        
def moveForward():    
    for i in range(4):
        wheels[i].setVelocity(2.0)

def stop():
    print("stopping")
    for i in range(4):
        wheels[i].setVelocity(0.0)

        
def turnLeft(goal):
    reading = int(1000*(lidar_front.getRangeImage()[205]))
    goal = int(goal*1000)
    if reading == goal:
        for i in range(4):
            wheels[i].setVelocity(0.0)
            return true
    else :
        wheels[0].setVelocity(0.0)
        wheels[1].setVelocity(0.0)
        wheels[2].setVelocity(2.0)
        wheels[3].setVelocity(2.0)
               
while robot.step(TIME_STEP) != -1:

    if obstacleInfront() :
        stop()
        turn_right, distance = maxDistance()
        
        if turn_right :
            turnRight(distance)
        else :
            turnLeft(distance)
        
    else: 
        moveForward()
    
    
    
    
    
    
    
    
    
    