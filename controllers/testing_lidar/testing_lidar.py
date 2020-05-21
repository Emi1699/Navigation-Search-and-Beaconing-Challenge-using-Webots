from controller import Robot, Lidar

TIME_STEP = 64
robot = Robot()

wheels = []
wheelsNames = ['frontLeft', 'backLeft', 'frontRight', 'backRight']
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


def removeOne(list):
    new_list = []
    for i in list:
        if i != 1.0:
            new_list.append(i)
            
    return new_list
            

def obstacleInfront():
    if min(lidar_front.getRangeImage())<0.35 and min(lidar_front.getRangeImage())!=0 :
        return True
    else:
        return False
        
def maxDistance():
    right_val = lidar_right.getRangeImage()
    left_val = lidar_left.getRangeImage()
 
    new_right_val = removeOne(right_val)
    new_left_val = removeOne(left_val)
    
    right_max = max(new_right_val)
    left_max = max(new_left_val)
        
    print("right max", right_max)
    print("left max", left_max)
    print("@@@@@@@@")
    if right_max>left_max:
        return True, right_max
    else:
        return False, left_max
        
def turnRight(goal):
    reading = int(10000*(lidar_front.getRangeImage()[255]))
    goal = int(goal*10000)
    if reading == goal:
        stop()
        return True
    else :
        wheels[0].setVelocity(0.0)
        wheels[1].setVelocity(0.0)
        wheels[2].setVelocity(0.0)
        wheels[3].setVelocity(0.0)
        
    i = 0
    while i != 100 :
        i += 1
        
    print(i)
        
def moveForward():    
    for i in range(4):
        wheels[i].setVelocity(5.0)

def stop():
    for i in range(4):
        wheels[i].setVelocity(0.0)

        
def turnLeft(goal):
    reading = int(10000*(lidar_front.getRangeImage()[255]))
    goal = int(goal*10000)
    
    if reading == goal:
        stop()
        return True
    else :
        wheels[0].setVelocity(0.0)
        wheels[1].setVelocity(0.0)
        wheels[2].setVelocity(5.0)
        wheels[3].setVelocity(5.0)      
               
while robot.step(TIME_STEP) != -1:
    
    # if obstacleInfront() :
        # stop()
        # print("calling maxD")
        # turn_right, distance = maxDistance()
        # reading = int(10000*(lidar_front.getRangeImage()[255]))
        # goal = int(10000*distance)
        
        # if turn_right :
            # turnRight(distance)
        # else :
            # turnLeft(distance)
        
        # if reading == goal:
            # moveForward()
        
    # else: 
        # moveForward()
        turnRight(3)
    
    
    
    
    
    
    
    
    
    