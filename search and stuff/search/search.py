from controller import Robot, Lidar

TIME_STEP = 64
robot = Robot()

gps = robot.getGPS("gps")
gps.enable(TIME_STEP)

wheels = []
wheelsNames = ['wheelleft1', 'wheelleft2', 'wheelright1', 'wheelright2']
for i in range(4):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

lidar = robot.getLidar("lidar_front")
lidar.enable(TIME_STEP)

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
    print("posx ",pos_x, " posz ",pos_z)
    print("x ",curr_x, " z ",curr_z)
    if ( curr_x<(pos_x+1.0) and 
         curr_x>(pos_x-1.0) and 
         curr_z<(pos_z+1.0) and 
         curr_z>(pos_z-1.0) ):
        return True
    else:
        return False

startColour = [0,0.5,0]

turnRight = False
turnLeft = False
stop = False    
start_zone = True 
only_once = 0  
while stop == False:
   
    if robot.step(TIME_STEP) == -1:
        print("exited with -1")
        stop = True
    if only_once == 0:
        pos_x = gps.getValues()[0]
        pos_z = gps.getValues()[2]
        only_once = 1
        
    frontDistance = lidar.getRangeImage()[255]
    front_obs = (min(lidar_right.getRangeImage()) < 0.2)
    right_obs = (min(lidar_right.getRangeImage()) < 0.2)
    left_obs = (min(lidar_left.getRangeImage()) < 0.2)
    if isStartZone() == False:
        start_zone = False
        
    if not(right_obs or left_obs or front_obs or isStartZone()) :
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
        
            front_obstacle = (lidar.getRangeImage()[255] < 0.4)
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
        print("in here")
        leftSpeed = 3.0
        rightSpeed = 3.0
        
        front_obstacle = (min(lidar.getRangeImage()) < 0.35)
        right_obstacle = (min(lidar_right.getRangeImage()) < 0.15)
        left_obstacle = (min(lidar_left.getRangeImage()) < 0.15)
        
        if front_obstacle == False:
            turn = False
        else:
            turn = True
               
        if (right_obstacle or left_obstacle) and (not front_obstacle):
            leftSpeed = 3.0
            rightSpeed = 3.0
            
        elif front_obstacle:
            if turn:
                leftSpeed = 1.0
                rightSpeed = -1.0
                
        elif left_obstacle:
            # turn right
            leftSpeed  = 1.0 
            rightSpeed = -1.0
            
        elif right_obstacle:
            # turn left
            leftSpeed  = -1.0 
            rightSpeed = 1.0            

        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(leftSpeed)
        wheels[2].setVelocity(rightSpeed)
        wheels[3].setVelocity(rightSpeed)        
        
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
    
    
    
    
    