from controller import Robot, DistanceSensor, Motor

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ds = []
dsNames = [
    'ds_centre', 'ds_right1', 'ds_right2', 'ds_right3',
    'ds_left1', 'ds_left2', 'ds_left3'
]

for i in range(7):
    ds.append(robot.getDistanceSensor(dsNames[i]))
    ds[i].enable(TIME_STEP)

wheels = []
wheelsNames = ['wheelleft1', 'wheelleft2', 'wheelright1', 'wheelright2']
for i in range(4):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    dsValues = []
    for i in range(7):
        dsValues.append(ds[i].getValue())
    # process behavior
    # detect obstacles
    #print(dsValues[0])
    centre_obstacle = dsValues[0] < 1000.0
    right_obstacle = dsValues[1] < 1000.0 or dsValues[2] < 1000.0 or dsValues[3] < 1000.0
    left_obstacle = dsValues[4] < 1000.0 or dsValues[5] < 1000.0 or dsValues[6] < 1000.0
    # write actuators inputs
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if centre_obstacle:
        leftSpeed  -= 0.5 * MAX_SPEED
        rightSpeed -= 0.5 * MAX_SPEED
        #wall infront
        if left_obstacle:
            # turn right
            leftSpeed  += 0.25 * MAX_SPEED
            rightSpeed -= 0.5 * MAX_SPEED
        elif right_obstacle:
            # turn left
            leftSpeed  -= 0.5 * MAX_SPEED
            rightSpeed += 0.25 * MAX_SPEED
    # write actuators inputs
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(leftSpeed)
    wheels[2].setVelocity(rightSpeed)
    wheels[3].setVelocity(rightSpeed)
