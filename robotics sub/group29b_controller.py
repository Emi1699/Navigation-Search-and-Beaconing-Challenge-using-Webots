from controller import Robot, Lidar

TIME_STEP = 64
robot = Robot()

#setting up the devices
gps = robot.getGPS("gps")
gps.enable(TIME_STEP)

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

camera_front = robot.getCamera("camera_front")
camera_front.enable(TIME_STEP)
camera_front.recognitionEnable(TIME_STEP)

camera_left = robot.getCamera("camera_left")
camera_left.enable(TIME_STEP)
camera_left.recognitionEnable(TIME_STEP)

camera_right = robot.getCamera("camera_right")
camera_right.enable(TIME_STEP)
camera_right.recognitionEnable(TIME_STEP)

################################################
#functions for the maze navigation
################################################
def removeOne(list):
    new_list = []
    for i in list:
        if i != 1.0:
            new_list.append(i)
            
    return new_list
            

def obstacleInfront(f):

    if min(f)<0.35 and min(f)!=0 :
        return True
    else:
        return False
        
def maxDistance(r,l):

    right_val = r
    left_val = l
 
    new_right_val = removeOne(right_val)
    new_left_val = removeOne(left_val)
    
    right_max = max(new_right_val)
    left_max = max(new_left_val)
        
    if (right_max+0.05)>left_max:
        return True, right_max
    else:
        return False, left_max
        
def turnRightf(goal):
    reading = int(100*(lidar_front.getRangeImage()[255]))
    goal = int(goal*100)
    if reading == goal:
        stopBot()
        return True
    else :
        wheels[0].setVelocity(3.0)
        wheels[1].setVelocity(3.0)
        wheels[2].setVelocity(0.0)
        wheels[3].setVelocity(0.0)
        
def moveForward():    
    for i in range(4):
        wheels[i].setVelocity(3.0)

def stopBot():

    for i in range(4):
        wheels[i].setVelocity(0.0)

        
def turnLeftf(goal):
    reading = int(100*(lidar_front.getRangeImage()[255]))
    goal = int(goal*100)
    
    if reading == goal:
        stopBot()
        return True
    else :
        wheels[0].setVelocity(0.0)
        wheels[1].setVelocity(0.0)
        wheels[2].setVelocity(3.0)
        wheels[3].setVelocity(3.0)      


stop = False
#######################################################
#functions for the avoid and wander
#######################################################

def colourInCentre(c):
    if camera_front.getRecognitionNumberOfObjects() != 0 :
        obj = camera_front.getRecognitionObjects()
        colour = obj[0].get_colors()
        if c == colour:
            i = obj[0].get_position_on_image()[0]
            j = obj[0].get_position_on_image()[1]
            if i>20 and i<43 and j>20 and j<43 :
                return True
            else:
                return False
        else:
            return False
    return False    

def getStartColour():
    if camera_right.getRecognitionNumberOfObjects() != 0 :
        obj = camera_right.getRecognitionObjects()
        colour = obj[0].get_colors()
        return colour
    else :
        return False

def foundColourFront(c):
    if camera_front.getRecognitionNumberOfObjects() != 0 :
        obj = camera_front.getRecognitionObjects()
        colour = obj[0].get_colors()
        if c == colour:
            return True
        else:
            return False
    return False

def foundColourRight(c):
    if camera_right.getRecognitionNumberOfObjects() != 0 :
        obj = camera_right.getRecognitionObjects()
        colour = obj[0].get_colors()
        if c == colour:
            return True
        else:
            return False
    return False

def foundColourLeft(c):
    if camera_left.getRecognitionNumberOfObjects() != 0 :
        obj = camera_left.getRecognitionObjects()
        colour = obj[0].get_colors()
        if c == colour:
            return True
        else:
            return False
    return False

def moveForward():    
    for i in range(4):
        wheels[i].setVelocity(3.0)
 
def stopBot():
    for i in range(4):
        wheels[i].setVelocity(0.0)     
 
def isStartZone():
    curr_x = gps.getValues()[0]
    curr_z = gps.getValues()[2]

    if ( curr_x<(pos_x+1.0) and 
         curr_x>(pos_x-1.0) and 
         curr_z<(pos_z+1.0) and 
         curr_z>(pos_z-1.0) ):
        return True
    else:
        return False

def isMaze():
    if ( (camera_front.getRecognitionNumberOfObjects() != 0) and 
         not(isStartZone())):
        return False
    else:
        return True
####################################################


startColour = []

turnRight = False
turnLeft = False
stop = False    
start_zone = True 
only_once = 0  
maze = True

####################################################
#control loop
####################################################
while stop == False:
   
    if robot.step(TIME_STEP) == -1:
        print("exited with -1")
        stop = True
    
    if only_once == 0:
        obj_c = camera_left.getRecognitionObjects()
        startColour = obj_c[0].get_colors()

        pos_x = gps.getValues()[0]
        pos_z = gps.getValues()[2]
        only_once = 1
        
    if maze == True:
        maze = isMaze()
        print(maze)
               
    frontDistance = lidar_front.getRangeImage()[255]
    front_obs = (min(lidar_front.getRangeImage()) < 0.15)
    right_obs = (min(lidar_right.getRangeImage()) < 0.15)
    left_obs = (min(lidar_left.getRangeImage()) < 0.15)
    if isStartZone() == False:
        start_zone = False
     
   
    if not(right_obs or left_obs or front_obs or isStartZone() or maze) :
        #this is the avoid and wander part of the code
        print("in avoid and wander")
        if foundColourFront(startColour):
            if colourInCentre(startColour):
                moveForward()
                
                if (frontDistance<0.3) and (foundColourFront(startColour)):
                    stopBot()
                    stop = True
            else:
                turnRight = True
      
        elif foundColourRight(startColour):
            turnRight = True
        
        elif foundColourLeft(startColour):
            turnLeft = True
        else:
            leftSpeed = 3.0
            rightSpeed = 3.0
        
            front_obstacle = (lidar_front.getRangeImage()[255] < 0.4)
            right_obstacle = (min(lidar_right.getRangeImage()) < 0.3)
            left_obstacle = (min(lidar_left.getRangeImage()) < 0.3)
        
            if left_obstacle and front_obstacle:
                # turn right
                leftSpeed  = 1.0 
                rightSpeed = -1.0
            if right_obstacle and front_obstacle:
                # turn left
                leftSpeed  = -1.0 
                rightSpeed = 1.0            
            if front_obstacle:
                l = min(lidar_left.getRangeImage())
                r = min(lidar_right.getRangeImage())
                if r>=l:
                    # turn right
                    leftSpeed  = 1.0 
                    rightSpeed = -1.0
                else:
                    # turn left
                    leftSpeed  = -1.0 
                    rightSpeed = 1.0

            wheels[0].setVelocity(leftSpeed)
            wheels[1].setVelocity(leftSpeed)
            wheels[2].setVelocity(rightSpeed)
            wheels[3].setVelocity(rightSpeed) 
    else:
        print("in maze navigation")
        #this is the maze navigation part of the code
        if robot.step(TIME_STEP) == -1:
            print("exited with -1")
            stop = True
            
        if not isMaze():
            flag = 0
        f = lidar_front.getRangeImage()
        r = lidar_right.getRangeImage()
        l = lidar_left.getRangeImage()
           
        if obstacleInfront(f) :
            stopBot()
            turn_right, distance = maxDistance(r,l)
            reading = int(100*(lidar_front.getRangeImage()[255]))
            goal = int(100*distance)
        
            if turn_right :
                turnRightf(distance)
            else :
                turnLeftf(distance)
        
            if reading == goal:
                moveForward()
        
        else:
            d = lidar_right.getRangeImage()[511]
            if d > 0.7:
                wheels[0].setVelocity(2.0)
                wheels[1].setVelocity(2.0)
                wheels[2].setVelocity(-2.0)
                wheels[3].setVelocity(-2.0)  
            else :
                moveForward()               
     ##   
    if turnRight:
        if foundColourFront(startColour) and colourInCentre(startColour):
            turnRight = False
        wheels[0].setVelocity(1.0)
        wheels[1].setVelocity(1.0)
        wheels[2].setVelocity(-1.0)
        wheels[3].setVelocity(-1.0)
        
    if turnLeft:
        if foundColourFront(startColour) and colourInCentre(startColour):
            turnLeft = False
        wheels[0].setVelocity(-1.0)
        wheels[1].setVelocity(-1.0)
        wheels[2].setVelocity(1.0)
        wheels[3].setVelocity(1.0)

    
    
    
    
    
    
    
    
    
    