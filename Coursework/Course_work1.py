# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import math
import sys
import random
# import matplotlib.pyplot as plt    #used for image plotting;
import numpy as np
from math import cos
from math import sin
from statistics import mean

#------------- Configulation Parameters -------------#
# Set status mission
MISSION = ["1 : Find Middle",
            "2 : Leave Room",
            "3 : Wall Explorer",
            "4 : Find Beacon", 
            "5 : Go Home","End Program"
            ]          #To keep track of prime directive

THRESHOLD_BEACON = 3          #Max distance that the beacon can be detected, set standard to 7
MIN_DISTANCE_BEACON=0.5       #Min distance to stany away from the Beacon (to take robot dimension into account)
MIN_DISTANCE_ROOM=0.05        #Tolerance with regards to moving to the middle of the room
MIN_FRONT_DISTANCE=0.2        #Min distance accepted with regards to forward distance to the wall
MIN_SIDE_DISTANCE=1           #Min distance accepted with regards to side distance to the wall
ANTI_COLLISION_DISTANCE=1.5   #Distance at which the anti-collision becomes active
SETPOINT = 2;                 #The desired distance from the wall the robot must follow in the wall follow algorithm
RUNTIME = 50;               #Time the small room following logic should run
ENDTIME = 750;               #Time the wall following logic should run
SPEED_ROBOT=3                 #Speed at which the robot drives, note that PID tuning must be done again if changed! 

###Testing turning interval in inner room
ANGLES=[0, math.pi/8, math.pi/4, 3*math.pi/8, math.pi/2, 5*math.pi/8, 3*math.pi/4, 7*math.pi/8]


class Robot():
    
    def __init__(self):
                
        # Setup Motors
        res, self.leftMotor = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
        res, self.rightMotor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)

        # Setup Robot
        res, self.robot = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking)
        

        # start position
        self.position = [] # array will hold the x,y and Phi of the robot
        res, self.fullPosition = sim.simxGetObjectPosition(clientID, self.robot, -1, sim.simx_opmode_blocking)
        res, self.nextAngles = sim.simxGetObjectOrientation(clientID, self.robot, -1, sim.simx_opmode_blocking)
        self.position.append(self.fullPosition[0])
        self.position.append(self.fullPosition[1])
        self.position.append(self.nextAngles[2])

        # Setup Sonars
        self.sonarHandles = [] 
        text = 'Pioneer_p3dx_ultrasonicSensor'

        for i in range(1,17):
            sonarText = text + str(i)
            res, tempHandle = sim.simxGetObjectHandle(clientID, sonarText, sim.simx_opmode_blocking)
            self.sonarHandles.append(tempHandle)

        # res, self.frontLeftSonar = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor5',sim.simx_opmode_blocking)
        # res, self.Front_30_RightSonar = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor6',sim.simx_opmode_blocking)
        # res, self.RightSonar = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor8',sim.simx_opmode_blocking)
        # res, self.backRightSonar = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor9',sim.simx_opmode_blocking)

         # Start Sonars
        for sonar in self.sonarHandles:
            res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sonar, sim.simx_opmode_streaming)
        # res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.frontLeftSonar,sim.simx_opmode_streaming)
        # res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.Front_30_RightSonar,sim.simx_opmode_streaming)
        # res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.RightSonar,sim.simx_opmode_streaming)
        # res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.backRightSonar,sim.simx_opmode_streaming)

      

        #start Beacon
        res, self.distHandle = sim.simxGetDistanceHandle(clientID, 'beaconDistance', sim.simx_opmode_blocking)
        res, distance = sim.simxReadDistance(clientID, self.distHandle, sim.simx_opmode_streaming)
        res, self.beaconHandle = sim.simxGetObjectHandle(clientID, 'beacon',sim.simx_opmode_blocking)

        #------------------------------------------#

        # PID Controller value
        # Kp = 2 # Proportional Gain
        # Ki = 0.05 # Integral Gain
        # Kd = 5 # Derivative Gain
        # Dt = 0.2 # Set Time stable

        self.objective = MISSION[0] # Set mission to Array
        self.antiCollisionDistance = ANTI_COLLISION_DISTANCE 
        self.minFrontDistance = MIN_DISTANCE_BEACON

        self.turnKp = 3.75 # Proportional Gain turn 
        self.turnKi = 0.006 # Integral Gain turn 
        self.turnKd = 6 #Derivative Gain turn 

        self.Kp = 1 # Proportional Gain Follow Wall
        self.Ki = 0 # Integral Gain Follow Wall
        self.Kd = 35 # Derivative Gain Follow Wall
        
        #------------------------------------------#

        # time.sleep(1)

    #Starting Sensors, front and back
    def getObjective(self):
        return self.objective

    def setObjective(self, objective):
        self.objective = objective

    def getDistanceReading(self, sonarID):
        # Get reading from sensor
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.sonarHandles[sonarID],sim.simx_opmode_buffer)

        if detectionState == 1:
            # return magnitude of detectedPoint
            return math.sqrt(sum(i**2 for i in detectedPoint))
        else:
            # resturn another value that we know cannon be true and handle it (use a large number so that if you do 'distance < reading' it will work)
            return 9999

    def getPosition(self):
        # Get the position form the robot && the world frame (-1 value sees )
        res, self.fullPosition = sim.simxGetObjectPosition(clientID, self.robot, -1, sim.simx_opmode_blocking)

        # Get the alignment of the robot && the world axis (-1 value sees)
        res, self.nextAngles = sim.simxGetObjectOrientation(clientID, self.robot, -1, sim.simx_opmode_blocking)
        self.position[0] = self.fullPosition[0]
        self.position[1] = self.fullPosition[1]
        self.position[2] = self.nextAngles[2]
        return self.position

    def stop(self):
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, 0, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, 0, sim.simx_opmode_blocking)

    def turn(self, turnVelocity):
        # turnVelocity < 0 = trun left
        # turnVelocity > 0 = turn right
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, turnVelocity, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, -turnVelocity, sim.simx_opmode_blocking)

    def curveCorner(self, leftMotorVelocity, rightMotorVelocity):
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, leftMotorVelocity, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, rightMotorVelocity, sim.simx_opmode_blocking)

    def move(self, velocity):
        # velocity < 0 = reverse
        # velocity > 0 = forward
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, velocity, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, velocity, sim.simx_opmode_blocking)

    def turnToDirection(self, targetAngle):
        # Method to turn the robot to a specific direction/heading based on PID
        # Input : direction / angle in rad to turn to PID
        current_error = adjustAngle(self.getPosition()[2] - targetAngle) # Currect Error
        previous_error = 0 # Previous Error
        sum_error = 0 # Sum Error (Integral)

        # Corrent turn as long as the differecnt is larger than 1/2 degree
        while abs(current_error) > math.pi/180: 
            sum_error = sum_error + current_error # Update the Sum error
            derivative = current_error - previous_error # Calculate the derivative
            previous_error = current_error # Update Previous error
            # PID Process Calculate control function
            cp = self.turnKp * current_error + self.turnKi * sum_error + self.turnKd * derivative

            self.curveCorner(cp, -cp)

            #calculate the current error
            current_error = adjustAngle(self.getPosition()[2] - targetAngle)

    def findDirection(self, x, y):
        # Find the angle between the current orientation of the robot and the target object
        # Input : target coordinates x and y || output : angle in rad

        # get the current position and oreientation
        xCurrent, yCurrent, phiCurrent = self.getPosition()

        # Calculate difference vector
        xDiff = x - xCurrent
        yDiff = y - yCurrent

        # Calculate target angle taking into account of tan -1
        if xDiff < 0 :
            phi = math.atan(yDiff / xDiff) - math.pi
        else:
            phi = math.atan(yDiff / xDiff)
        return phi

    def senseAll(self):
        # Pinging all sensor and returning the distances in a vector
        # Output : lisy of 16 distances
        sensor_readings = []

        for i in range(0, 16): sensor_readings.append(robot.getDistanceReading(i))
        return sensor_readings

    def findMiddleOfTheRoom(self):
        # finding the middle of the room
        # The middle is (x, y) maybe (0.05, 0.1) || output : (x,y)  coordinates of the middle of the room
        print("Searching the middle")
        x = 0.05
        y = 0.11025
        return (x, y)

    def goTo(self, x, y, tolerance):
        # Control to the location that you want
        # Input : x,y coordinates of target || output: Distance to object
        self.getPosition()

        while distance(x, y, self.position[0], self.position[1]) > tolerance : 
            # Determine the direction (x,y) lies and turn to it
            self.turnToDirection(self.findDirection(x, y))

            # ping the sonars
            sensor = self.senseAll()

            # if obstacle in front => avoid
            if (sensor[3] < ANTI_COLLISION_DISTANCE or 
                sensor[4] < ANTI_COLLISION_DISTANCE) and (self.findBeacon()[0] > tolerance or self.findBeacon()[0] < 0):
                
                while (sensor[3] < ANTI_COLLISION_DISTANCE or 
                sensor[4] < ANTI_COLLISION_DISTANCE or 
                sensor[2] < ANTI_COLLISION_DISTANCE or
                sensor[5] < ANTI_COLLISION_DISTANCE) and (self.findBeacon()[0] > tolerance or self.findBeacon()[0] < 0): 
                    vL,vR = robot.drive(SPEED_ROBOT,SPEED_ROBOT)
                    robot.curveCorner(vL,vR)
                    sensor = self.senseAll()

                while (sensor[8] < ANTI_COLLISION_DISTANCE or
                sensor[15] < ANTI_COLLISION_DISTANCE or
                sensor[1] < ANTI_COLLISION_DISTANCE or
                sensor[6] < ANTI_COLLISION_DISTANCE) :
                    self.move(SPEED_ROBOT)
                    sensor = self.senseAll()
            # otherwise move forward
            self.move(2)

    def drive(self,vL,vR):
        # receives the proposed wheel SPEED_ROBOTs but adjects for
        listReturnCodes = []
        listDetectionStates = []
        listDistances = []
        detect = []

        braitenbergL = [-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6]
        braitenbergR = [-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2]

        for i in range(0, 8):
            [returnCode, detectionState, distance, d1, d2] = sim.simxReadProximitySensor(clientID, self.sonarHandles[i], sim.simx_opmode_buffer)
            listReturnCodes.append(returnCode)
            listDetectionStates.append(detectionState)
            listDistances.append(distance)
            if detectionState == 1 and np.linalg.norm(distance) < self.antiCollisionDistance :
                if np.linalg.norm(distance) < self.minFrontDistance:
                    distance = self.minFrontDistance
                detect.append(1 - ((np.linalg.norm(distance) - self.minFrontDistance) / (self.antiCollisionDistance - self.minFrontDistance)))
            else :
                detect.append(0)

        vLeft = vL
        vRight = vR

        for i in range(0,8) :
            vLeft = vLeft + braitenbergL[i] * detect[i]
            vRight = vRight + braitenbergR[i] * detect[i]

        return vLeft,vRight

    def findBeacon(self, thresholdbeacon = THRESHOLD_BEACON):
        # Input : MAX distance of beacon that can be detected
        # Output : distance and position of beacon (array of 3 values)

        # Maxinum Distance the robot can sense the beacon
        threshold = thresholdbeacon

        # Get the distance to the beacon
        res, distance = sim.simxReadDistance(clientID, self.distHandle, sim.simx_opmode_buffer)
        # GET beacon position in x,y,z cube relative to world
        res, beaconPosition = sim.simxGetObjectPosition(clientID, self.beaconHandle, -1, sim.simx_opmode_blocking)

        # return distance and position if beacon is in range
        if distance >= threshold:
            return -1, [], threshold
        else:
            return distance, beaconPosition, threshold

#------------  Create Robot2 Class Use Values From Robot ---------#

class Robot2(Robot):
    
    def scan(self):
        # Keep of detected points in absolute / world reference
        xy = []

        for sonar in self.sonarHandles[0:16]:   # all sonar            
            # ping sonar
            res, detectionState, detectedPoint,detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sonar, sim.simx_opmode_buffer)

            # If a point is detected
            if detectionState==1:
                # Get the sensor orientation in next Angle for the sensor
                res, nextAngles = sim.simxGetObjectOrientation(clientID, sonar, -1, sim.simx_opmode_blocking)

                # Get position for the sonar VS the world frame (-1 values = sees)
                res, position = sim.simxGetObjectOrientation(clientID, sonar, -1, sim.simx_opmode_blocking)

                # Rotate the coordinates of the found point to receive absolute coordinates (- the offset of the sensor location)
                v1 = rotate(detectedPoint[0], detectedPoint[1], detectedPoint[2], nextAngles[0], nextAngles[1], nextAngles[2])

                # Add x and y coordinates of the detected point plus the location of the sensor to get absolute coordinates
                xy.append((position[0] + v1[0], position[1] + v1[1]))

        return xy

    def scanAround(self):
            # return function list of coordinates detected around the robot when turning around its axis in pi/8 degree increase
            xy = []
            for angle in ANGLES:
                self.turnToDirection(angle)
                xy += self.scan()

            return xy

    def smallroomFollow(self):
        print("Searching the middle")
        current_error = 0
        previous_error = 0
        sum_error = 0

        for i in range(RUNTIME):
            # Scan around every 30 ticks
            if i % 30 == 0:
                # Stop the require from the last driving = operates too long
                self.stop()
                map.addPoints(self.scan())
                
            # Ping sonars
            distanceFront = self.getDistanceReading(4)
            distanceRight = self.getDistanceReading(8)

            # Setting the front flag
            if (distanceFront <= SETPOINT): # object in front => turn left
                frontFlag = True
            else :
                frontFlag = False  # Don't have in front => keep running

            # Seting the Right Flag
            if distanceRight == 9999 :
                rightFlag = False  # Don't have in right => turn right
            else :
                rightFlag = True  # When detected on the right => robot will follow the wall

            if frontFlag :
                if distanceFront < MIN_FRONT_DISTANCE :
                    # Robot will stuck, back out random
                    self.curveCorner(-random.random(), -random.random())
                    time.sleep(0.1)
                else :
                    # turn left inplace
                    self.curveCorner(0,SPEED_ROBOT)

            elif rightFlag :
                # PID calculate the current error
                current_error = distanceRight - SETPOINT
                # Update Sum error
                sum_error = sum_error + current_error
                # Calculate the derivative
                derivative = current_error - previous_error
                # Updaet the previous error
                previous_error = current_error
                # calculate control function
                cp = self.Kp * current_error + self.Ki * sum_error + self.Kp * derivative

                self.curveCorner(SPEED_ROBOT,SPEED_ROBOT-cp)

            else:
                # turn right inplace because the wall sloped away
                self.curveCorner(SPEED_ROBOT, 0)

    def wallFollow(self):
        current_error = 0
        previous_error = 0
        sum_error = 0

        for i in range(ENDTIME):
            # Scan around every 30 ticks
            if i % 30 == 0:
                # Stop the require from the last driving = operates too long
                self.stop()
                map.addPoints(self.scan())
                
            # Ping sonars
            distanceFront = self.getDistanceReading(4)
            distanceRight = self.getDistanceReading(8)

            # Setting the front flag
            if (distanceFront <= SETPOINT): # object in front => turn left
                frontFlag = True
            else :
                frontFlag = False  # Don't have in front => keep running

            # Seting the Right Flag
            if distanceRight == 9999 :
                rightFlag = False  # Don't have in right => turn right
            else :
                rightFlag = True  # When detected on the right => robot will follow the wall

            if frontFlag :
                if distanceFront < MIN_FRONT_DISTANCE :
                    # Robot will stuck, back out random
                    self.curveCorner(-random.random(), -random.random())
                    time.sleep(0.1)
                else :
                    # turn left inplace
                    self.curveCorner(0,SPEED_ROBOT)

            elif rightFlag :
                # PID calculate the current error
                current_error = distanceRight - SETPOINT
                # Update Sum error
                sum_error = sum_error + current_error
                # Calculate the derivative
                derivative = current_error - previous_error
                # Updaet the previous error
                previous_error = current_error
                # calculate control function
                cp = self.Kp * current_error + self.Ki * sum_error + self.Kp * derivative

                self.curveCorner(SPEED_ROBOT,SPEED_ROBOT-cp)

            else:
                # turn right inplace because the wall sloped away
                self.curveCorner(SPEED_ROBOT, 0)

#----------------------  Create Map1 Class ---------------------------#

class Map1() :
    def __init__(self) :
        # Parameters of the map
        self.wayPoints = []  # list of waypoints in (x,y) coordinates

    def getWayPoint(self, number) :
        return self.wayPoints[number]

    def getWayPoints(self) :
        return self.wayPoints

    def addWayPoint(self, wayPoint) :
        self.wayPoints.append(wayPoint)

        #---------  Create Map2 Class Use Values From Map1()---------#
class Map2(Map1) :
    def __init__(self):
        # Upgrade Map1 > continue from map
        super().__init__()
        self.points = []  # List detected all point
        self.map = np.zeros((2, 2))   # creating a dummy 2x2 matrix

    def addPoints(self, points) :
        self.points += points

    def getPoints(self) :
        return(self.points)

    def printMap(self) :
        # Print XY scatter map of detected Points
        coord = list(zip(* self.getPoints()))

        # Building the plot
        plt.clf()
        plt.ylabel('Y-Axis')
        plt.xlabel('X-Axis')
        plt.title("Map of the Room")
        plt.scatter(coord[0], coord[1])
        plt.show()

    def createMap(self, resolution):
        # Input : points is list of (x, y) coordinates
        # Input : resolution is the size of an individual square of the map in m

        # splitting the points in 2 list of x and y
        coord = list(zip(* self.getPoints()))

        # Calculating the range and number of grid squares
        Xmax = int((max(coord[0]) - min(coord[0])) // resolution + 1) # range X
        Ymax = int((max(coord[1]) - min(coord[1])) // resolution + 1) # range Y

        self.max = np.zeros((Ymax, Xmax)) # create empty grid

    def updateMap(self, points, resolution):
        # Input : points is list of (x, y) coordinates
        # Input : resolution is the size of an individual square of the map in m

        # splitting the points in 2 list of x and y
        coord = list(zip(* self.getPoints()))

        # Calculating the range and number of grid squares
        xmin = min(coord[0]) # lower x value
        ymin = min(coord[1]) # lower y value

        for point in points:
            x = int((point[0] - xmin) // resolution)
            y = int(-(point[1] - ymin) // resolution)
            self.map[y][x] = 1

    def printGrid(self):
        plt.clf()
        plt.imshow(self.map)
        plt.show

#------------- Extra function -----------#

def adjustAngle(angle) :
    # return angles with circles added
    while angle > math.pi : angle -= math.pi*2
    while angle <= -math.pi : angle += math.pi*2
    return angle            

def distance (x1, y1, x2, y2) :
    # calculate distance between 2 points in 2 dim
    # input : (x1, y1) coordinates point 1, (x2, y2) coordinates point 2
    return math.sqrt((x1 - x2) **2 + (y1 - y2) **2)

def rotationMatrix(a, b, g):
    # Return the rotation matrix using Tait-Bryan angles alpha, beta and gamma
    rot = np.array([(cos(b) * cos(g), 0, sin(b)),

                    (cos(g) * sin(a) * sin(b), 0, -cos(b) * sin(a)),

                    (0, cos(g) * sin(a), 0)], dtype = float)
    return rot

def rotate(x, y, z, a, b, g):
    # Input : x y z Into -> sensor frame of reference
    # output : vector of x y z that they are rotation x y z
    v1 = np.array([(x), (y), (z)])
    v2 = np.matmul(rotationMatrix(a, b, g), v1)
    return v2

#------------------ Start Mission ------------------#

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')
    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    print()
    map = Map2()
    print(" Create Map ")
    print()

    # robot = Robot()
    robot = Robot2()
    print("robot - Mission : ", robot.getObjective())
    print("robot - location : ",robot.getPosition())
    # # Now try to retrieve data in a blocking fashion (i.e. a service call):
    # for i in range(10):
    #     print(robot.getDistanceReading(robot.frontLeftSonar))
    #     print(robot.getDistanceReading(robot.Front_30_RightSonar))
    #     print(robot.getDistanceReading(robot.RightSonar))
    #     print(robot.getDistanceReading(robot.backRightSonar))
        
        # time.sleep(0.2)

    # Function used to return a random integer within the range of 0 and 100 for the random wander
    randomNumber = random.randint(0, 100)

    while robot.getObjective() != "End Program": 
        # the robot will move untill it detects an object
        robot.move(1)


#------------------ Mission 1 Find the middle ------------------#
        
        if robot.getObjective() == MISSION[0]: # Mission 1 : Find the middle of the small room
            # robot.smallroomFollow()

            middle = robot.findMiddleOfTheRoom()

            robot.goTo(middle[0], middle[1], MIN_DISTANCE_ROOM)

            # Modift 1 - scanning the smallroom
            map.addPoints(robot.scanAround())

            robot.setObjective(MISSION[1])
            # Add waypoint to the map
            map.addWayPoint(middle)
            print("Mission 1 Complete - Middle of the room Arrived")
            print("The middle : ", map.getWayPoints())
            print()

            print("robot - Mission : ", robot.getObjective())
            print("robot - location : ",robot.getPosition())
            print("Current map : ", map.getWayPoints())
            

#------------------ Mission 2 Leave of the room ------------------#

        elif robot.getObjective() == MISSION[1]: # Mission 2 : Leave of the small room
            sensor = robot.senseAll() # Sensor contrains a list with readings from all sensors
            while sensor[3] != 9999 or sensor[4] != 9999: # turn toward the opening
                robot.turn(0.5)
                sensor = robot.senseAll()

            # leave the room
            while sensor[0] != 9999 or sensor[15] != 9999 or sensor[7] != 9999 or sensor[8] != 9999:
                robot.move(SPEED_ROBOT)
                sensor = robot.senseAll()

                # Modify 2 - scanning while driving
                map.addPoints(robot.scan())

            time.sleep(1)
            robot.stop

            # Adding second waypoint to map
            currentPosition = robot.getPosition()
            map.addWayPoint([currentPosition[0], currentPosition[1]])

            # Modify 3 - scanning the second waypoint
            map.addPoints(robot.scan())

            robot.setObjective(MISSION[2])
            print("Mission 2 Complete - robot leave of the room")
            print()
            print("robot - Mission : ", robot.getObjective())
            print("robot - location : ",robot.getPosition())
            print("Current map : ", map.getWayPoints())

#------------------ Mission 3 Wall Explorer and Scan ------------------#

        elif robot.getObjective() == MISSION[2]: # Mission 3 : Wall Explorer and scanning
            while robot.getDistanceReading(4) > SETPOINT:
                robot.move(SPEED_ROBOT)

            robot.wallFollow()
            robot.setObjective(MISSION[3])
            print("Mission 3 Complete - Scan Room complete")
            print()
            print("robot - Mission : ", robot.getObjective())
            print("robot - location : ",robot.getPosition())
            print("Current map : ", map.getWayPoints())

#------------------ Mission 4 Random around beacon ------------------#
        
        elif robot.getObjective() == MISSION[3]: # Mission 4 Random around beacon
            beaconSensing = robot.findBeacon()
            while beaconSensing[0] == -1: # while no beacon is detected
                vL, vR = robot.drive(SPEED_ROBOT,SPEED_ROBOT)
                robot.curveCorner(vL,vR)
                beaconSensing = robot.findBeacon()

                # Modify 4 - scanning while driving
                map.addPoints(robot.scan())
                
            print("Beacon Detected")
            # Add waypoint of Beacon to the map
            currentPosition = robot.getPosition()
            map.addWayPoint([currentPosition[0], currentPosition[1]])
            print("Current map : ", map.getWayPoints())
            # Tolerance is the distance from the center -> it is not edge of the robot to center beacon
            robot.goTo(beaconSensing[1][0], beaconSensing[1][1], MIN_DISTANCE_BEACON) 
            robot.setObjective(MISSION[4])
            currentPosition = robot.getPosition()
            map.addWayPoint([currentPosition[0], currentPosition[1]])
            print("Mission 4 Complete - found Beacon")
            print()
            print("robot - Mission : ", robot.getObjective())
            print("robot - location : ",robot.getPosition())
            print("Current map : ", map.getWayPoints())
            time.sleep(5)
            robot.stop()

#------------- Mission 5 Go back to center of the small room -------------#
        
        elif robot.getObjective() == MISSION[4]: # Go back to center of the small room
            waypoints = map.wayPoints.copy() # Copt the waypoint
            waypoints.pop() # Get rib of last waypoint because the robot is done

            for waypoint in range(len(map.wayPoints) -1) :
                target = waypoints.pop()
                robot.goTo(target[0], target[1], MIN_DISTANCE_ROOM)
                print("Waypoint has arrived : ", target[0], " , ", target[1])

            # Show the result that the robot complete 
            print("Mission 5 Complete - robot go back to center of the small room")
            print()

            robot.setObjective(MISSION[5])
    




    #------------------------------------------#

    # # Variables
    # cp = 0
    # current_error = 0
    # previous_error = 0
    # sum_error = 0
    # previous_error_derivative = 0
    # current_error_derivative = 0

    # #------------------------------------------#

    # # Distance variables
    # d_R_front = robot.getDistanceReading(robot.RightSonar)
    # d_R_back = robot.getDistanceReading(robot.backRightSonar)

    # #calculate distance
    # dist_robot_and_wall = d_R_front + d_R_back / 2 # distance between robot and wall
    # angle = d_R_front - d_R_back # angel of the wall
    # dist_to_wall = math.cos(angle) == (d_R_front / d_R_back) # desired distance between robot and wall
    
    #------------------------------------------#

    # Loop execution (nested while statement)
 
        # # Process Coding
        # cp = max(min(0.0, cp), 1.0) # limiting the output of values 0 - 1
        # cp = abs(cp)            # used to make any output value a positive value
        # print("error {} cp {} distance {}".format(current_error, cp, dist_robot_and_wall))
        
        #------------------------------------------#

        # if...elif ..else statements that allows us to check for multiple expressions
        # if robot.getDistanceReading(robot.frontLeftSonar) <= 1: #threshold value
        #     robot.curveCorner(-cp, cp) # velocity adjectment
        #     print("Wall detected in (L)front cp")  # the robot will turn if a wall is detected

        # elif robot.getDistanceReading(robot.Front_30_RightSonar) <= dist_to_wall:
        #     robot.curveCorner(-cp, cp)  # velocity adjustment
        #     print("Wall detected in Right_front_50_degrees") # the robot will trun if a wall is detected;

        # elif robot.getDistanceReading(robot.backRightSonar) == dist_to_wall and robot.getDistanceReading(robot.RightSonar) == dist_to_wall and detectionStateR == 1.0: # threshold value
        #     robot.move(1)   #velecity adjustment
        #     print("--------- [B-F] Perfectly Forward fix velocity ---------")   # the robot will turn to adjust direction

        # elif robot.getDistanceReading(robot.backRightSonar) <= dist_to_wall and detectionStateR == 1.0:
        #     robot.curveCorner(cp, angle)  # velocity adjustment
        #     print("fix position on the Right (1)") # the robot will turn if detected on the right within the threshold value
       
        # elif robot.getDistanceReading(robot.backRightSonar) >= dist_to_wall and detectionStateR == 1.0:
        #     robot.curveCorner(angle, cp)  # velocity adjustment
        #     print("fix position on the Left (angle,cp)") # the robot will turn to adjust direction

        # elif robot.getDistanceReading(robot.backRightSonar) >= dist_to_wall:
        #     robot.curveCorner(cp, cp)  # velecity adjustment
        #     print("Maintain position on the Left (cp, cp)") # the robot will turn to adjust direction 

        # elif robot.getDistanceReading(robot.Front_30_RightSonar) <= dist_to_wall and detectionStateR == 1.0:
        #     robot.curveCorner(cp, angle)
        #     print("Change direction to the Right")

        # elif robot.getDistanceReading(robot.Front_30_RightSonar) >= dist_to_wall and detectionStateR == 1.0:
        #     robot.curveCorner(angle, cp)
        #     print("fix position to the Left #1")    

        # elif robot.getDistanceReading(robot.Front_30_RightSonar) >= dist_to_wall :
        #     robot.curveCorner(cp, cp)
        #     print("fix position to the Left #2")

        # elif robot.getDistanceReading(robot.RightSonar) == dist_to_wall and robot.getDistanceReading(robot.backRightSonar) == dist_to_wall and detectionStateR == 1.0:
        #     robot.move(1)
        #     print("--------- [F-&-D] Perfectly forward to fix velocity ---------")   

        # elif robot.getDistanceReading(robot.RightSonar) <= dist_to_wall and detectionStateR == 1.0:
        #     robot.curveCorner(cp, angle)
        #     print("Change direction to the Right")  

        # elif robot.getDistanceReading(robot.RightSonar) >= dist_to_wall and detectionStateR == 1.0:
        #         robot.curveCorner(angle, cp)
        #         print("fix position to the Left")

        # elif robot.getDistanceReading(robot.RightSonar) >= dist_to_wall :
        #         robot.curveCorner(cp, cp)
        #         print("fix position to the Left")      

    


        # elif robot.getDistanceReading(robot.RightSonar) <= 0.40:
        #     # robot.stop()
        #     # robot.curveCorner(0.25, 0.35)             
        #     robot.curveCorner(0.85, 1)             
        #     # robot.move(0.5)           
        #     print("0.40 Change direction to the Left") # the robot will turn to adjust direction;

        # elif robot.getDistanceReading(robot.RightSonar) <= 0.45:
        #     # robot.stop()
        #     # robot.curveCorner(0.25, 0.35)             
        #     robot.curveCorner(2, 0.15)             
        #     # robot.move(0.5)           
        #     print("0.45 Change direction to the Left") # the robot will turn to adjust direction;

        # elif robot.getDistanceReading(robot.RightSonar) <= 0.50:
        #     # robot.stop()
        #     # robot.curveCorner(0.40, 0.30)    
        #     robot.curveCorner(0.85, 1)    
        #     # robot.move(0.5)                    
        #     print("0.50 Change direction to the Left") # the robot will turn to adjust direction;
            
        # elif robot.getDistanceReading(robot.RightSonar) <= 0.60:
        #     # robot.stop()
        #     # robot.curveCorner(0.35, 0.25)                        
        #     robot.curveCorner(2, 0.25)                        
        #     print("0.60 Change direction to the Left") # the robot will turn to adjust direction;

        # elif robot.getDistanceReading(robot.RightSonar) <= 2:
        #     robot.stop()
        #     robot.curveCorner(3, 0.1)                        
        #     print("Big TurnRight") # the robot will turn to adjust direction;

        # elif robot.getDistanceReading(robot.backRightSonar) <= 2:
        #     robot.curveCorner(3, 0.1)
        #     print("Searching the wall")
        #     # print(robot.getDistanceReading(robot.backRightonar))

        # else: 
        #     robot.curveCorner(randomNumber - 1, randomNumber + 1) # The robot will take a random turn untill it detects an object;
        robot.stop()

else:
    print ('Failed connecting to remote API server')
print ('Program ended')
sys.exit('Disconnected')