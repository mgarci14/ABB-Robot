import abb
import time
import datetime
from pyquaternion import Quaternion
import math
import random
import csv
import os
import cv2






class moveableObj:
    """A class for movable objects in our workspace. Tracks X,Y,Z position and rotation of objects, and the tool they are used to interact with"""
    def __init__(self, initLoc, initIsRot, initRot, toolData, toolMinMax, tool_command, initSpeed, radius, initApproachAngle, initHeight):

        #Tracks the current location of the cylinder in the world plane as [X, Y, Z] and its rotation relative to the quaternion of it's tool [q1,q2,q3,q4]
        self.cartLoc = initLoc

        #Tracks the degree and distance value given by the random number generator [deg, distance]
        self.polarLoc = [0.0,self._getInitPolar()]

        #Refers to the joint positions of the robot at the location of the object
        self.jointLoc = [0,0,0,0,0,0]

        #Bool value that determines if object should have random rotational values
        self.isRot = initIsRot

        #relative to the world, the degree value of rotation with respect to the XY plane
        self.rot = initRot

        #Establishes the tool used to interact with object
        self.tool = toolData

        #Stores the min and max polar coord value for the given tool
        self.toolMinMax = toolMinMax

        #Refers to the Set_Do command associated with the tool
        self.activate_tool = tool_command

        #Refers to the maximum lift speed of the object
        self.maxLiftSpeed = initSpeed

        #Refers to the loose radial bounding box around the centerpoint of the object
        self.radialEdge = radius

        #Refers to the angle at which the object should be approched
        self.approachAngle = initApproachAngle

        #Refers to the height of the object
        self.height = initHeight

        #Refers to the z modifier
        self.zMod = 0

        #Boolean value for if the move object is currently grabbed
        self.isMoving = False
        
    #adjusts the approach angle in the cartesian coords
    def getApproachQuat(self, modQuat):
        if self.approachAngle == 0:
                return modQuat
        else:
            q1 = Quaternion(modQuat)
            q2 = Quaternion(axis = [0.0,1.0,0.0], degrees = self.approachAngle)

            q3 = q1 * q2

            return list(q3)
    #method for determining initial polar loc
    def _getInitPolar(self):
        '''Input [X,Y,Z] location and return the angle necessary to align axis 1 with the object'''
        angle = 0
        loc = self.cartLoc[0]
        if(loc[1] != 0):
            angle = math.degrees(math.atan(abs(loc[1])/abs(loc[0])))
            if(loc[1] < 0):
                angle = angle*(-1)


        return angle

    def updateObjectData(self, R):
        '''Method for updating object data'''
        return

    


class DatasetCollector:
    def __init__(self, total_photos, save_path):
        self.total_photos = total_photos
        self.save_path = save_path
        self.csv_file = os.path.join(save_path, 'dataset.csv')
        
        #print("Open CV")
        self.cap = cv2.VideoCapture(0)
        #print("Open CV Success")
        self.cap.set(3, 1920)
        self.cap.set(4, 1080)
        #check if output dir is there and create if not
        if not os.path.exists(save_path):
            os.makedirs(save_path)
        
            # Initialize CSV file
            with open(self.csv_file, 'w', newline='') as file:
                writer = csv.writer(file)
                #add data columns here
                writer.writerow(["Photo Name", "cyl.X", "cyl.Y", "cyl.Z", "cyl.q1", "cyl.q2", "cyl.q3", "cyl.q4",
        "cyl.r", "cyl.deg", "cyl.j1", "cyl.j2", "cyl.j3", "cyl.j4", "cyl.j5",
        "cyl.j6", "cyl.rot", "sqr.X", "sqr.Y", "sqr.Z", "sqr.q1", "sqr.q2",
        "sqr.q3", "sqr.q4", "sqr.r", "sqr.deg", "sqr.j1", "sqr.j2", "sqr.j3",
        "sqr.j4", "sqr.j5", "sqr.j6", "sqr.rot"])
        self.photo_count = len(os.listdir(self.save_path)) -1

    def capture_photo(self):
        self.photo_count += 1
        photo_name = self._generate_photo_name()
        photo_path = os.path.join(self.save_path, photo_name)


        ret, frame = self.cap.read()
        if ret:
            cv2.imwrite(photo_path, frame)

        return photo_name

    def update_csv(self, photo_name, objectList):
        row = []
        row.append(photo_name)

        for moveObj in objectList:
            row.append(moveObj.cartLoc[0][0])
            row.append(moveObj.cartLoc[0][1])
            row.append(moveObj.cartLoc[0][2])
            row.append(moveObj.cartLoc[1][0])
            row.append(moveObj.cartLoc[1][1])
            row.append(moveObj.cartLoc[1][2])
            row.append(moveObj.cartLoc[1][3])

            row.append(moveObj.polarLoc[0])
            row.append(moveObj.polarLoc[1])

            row.append(moveObj.jointLoc[0])
            row.append(moveObj.jointLoc[1])
            row.append(moveObj.jointLoc[2])
            row.append(moveObj.jointLoc[3])
            row.append(moveObj.jointLoc[4])
            row.append(moveObj.jointLoc[5])

            row.append(moveObj.rot)

        with open(self.csv_file, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(row)



    def _generate_photo_name(self):
        date_str = datetime.datetime.now().strftime("%d%H%M")
        return f"{date_str}_{self.photo_count}_{self.total_photos}.jpg"

    #Counts the amount of photos already taken. Returns the number of photos
    def count_init_data(self):
        files = os.listdir(self.save_path)
        self.photo_count = len(files) - 1
        return self.photo_count
        

    def close_class(self):
        self.cap.release()

class metricTracker:
    """A Class for stat tracking and time calculation"""
    def __init__(self, save_path, starting_iterations = 0):
        self.start_time = 0
        self.loop_time = 0
        self.total_time = 0
        self.iteration_count = starting_iterations
        self.run_count = 0
        self.fastest_time = float('inf')
        self.slowest_time = 0
        self.csv_filename = os.path.join(save_path, 'metric_log.csv')
        self.create_csv_file()

    def create_csv_file(self):
        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Iteration", "Previous Time", "Average Time", "Fastest Time", "Slowest Time", "Total Time"])

    def start_timer(self):
        self.start_time = time.time()

    def end_timer(self):
        end_time = time.time()
        self.loop_time = end_time - self.start_time
        self.update_metrics(self.loop_time)

    def update_metrics(self, loop_time):
        self.total_time += loop_time
        self.iteration_count += 1
        self.run_count += 1
        self.fastest_time = min(self.fastest_time, loop_time)
        self.slowest_time = max(self.slowest_time, loop_time)

    def get_average_time(self):
        return self.total_time / self.run_count if self.run_count else 0

    def estimate_remaining_time(self, total_iterations):
        average_time = self.get_average_time()
        remaining_iterations = total_iterations - self.iteration_count
        return remaining_iterations * average_time

    def output_to_csv(self):
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(
                [self.run_count, self.loop_time, self.get_average_time(), self.fastest_time, self.slowest_time, self.total_time])

    def format_time(self, seconds):
        hours = seconds // 3600
        minutes = (seconds % 3600) // 60
        seconds = seconds % 60
        return f"{int(hours)}h {int(minutes)}m {int(seconds)}s"



## GLOBAL VARS ##

robIP = '192.168.125.1'

#TOOLS
#tGripper = [[1.1,11.5,139.6],[-0.70711,0.70711,.70711,.70711]]
'''
#SIMULATION VALUES
tGripper = [[1.1,11.5,139.6],[0.5,-0.5,-0.5,-0.5]]
tGripper_adv = [1,[[13.941,-25.105,74.149],[1,0,0,0]]]

tVac = [[1.58, -47.25, 24.06],[0.5,-0.5,-0.5,-0.5]]
tVac_adv = [1,[[13.941,-25.105,74.149],[1,0,0,0]]]

RBS MEASSURE
tGripper = [[-2.9189,28.3118,135.933],[0.5,-0.5,-0.5,-0.5]]
tGripper_adv = [.5,[[2,0,61.2],[1,0,0,0]]]

tVac = [[], [0.5,-0.5,-0.5,-0.5]]
tVac_adv = [.5,[[2,0,61.2],[1,0,0,0]]]
'''

tGripper = [[0,13.71,142.91],[0.5,-0.5,-0.5,-0.5]]
tGripper_adv = [.5,[[2,0,61.2],[1,0,0,0]]]

tVac = [[0,-42.72,35.37],[0.5,-0.5,-0.5,-0.5]]
#tVac = [[0,-46.72,35.37],[0.44528,-.54805,-.54936,-.44672]]
tVac_adv = [.5,[[2,0,61.2],[1,0,0,0]]]

#Manual Joint rotations
jHomeTrue = [0,45,45,0,-90,90]
#Modified jHome with travel height integrated
jHome = [0.0, 14.03, 49.95, 0.0, -63.96, 90.0]
jZero = [0,0,0,0,0,0]
jOffScreen = [90,0,0,0,0,0]
jOffScreenNeg = [-90,0,0,0,0,0]

humanPos = [[566.8728879080437, 475.662831168039, 90], [0.94, 0.0, 0.0, 0.342]]

#Speed Data
'''
defaultSpeed = [550, 500, 5000, 1000]
moveSpeed = [800, 500, 5000, 1000]
squareLiftSpeed = [300,500,5000,1000]
'''

defaultSpeed = [300, 500, 5000, 1000]
moveSpeed = defaultSpeed
squareLiftSpeed = defaultSpeed


defaultZone = [0,0,0]



#Starting Positions
ketchupBottleStart = [[-337.2, 581.91, 90], [0.5, 0.0, 0.0, 0.866]]
mustardBottleStart = [[-428, 581.91, 90], [0.442, 0.0, 0.0, 0.897]]
travelHeight = 325

#Program Parameters
picCount = 2000


#Spacing and Overlap Parameters
"""
General Sizing Notes:
Table = 48x49, 32 inches high (1219.2, 1244.6, 812.8 (mm))
Red Square = 7x7 (177.8 x 177.8)
cylinder whole = 2x6 (50.8 x 152.4)
non-top-hat-cylinder = 2x4 (50.8, 101.6)
top-hat-cylinder = 2x4 (50.8, 101.6)
arm = 27 high 31 wide

"""
armWidth = 200
#longMinDegreeSep = 11.2
#shortMinDegreeSep = 25

bottleRadialEdge = 29.25


bottleHeight = 90
bottleApproachAngle = 0

minSeperation = 20

maxMoveDistance = 500
minMoveDistance = 150

#Random number generator parameters
#Determined by tool size and reachable position relative to the table.
#ROTATIONAL VALUES: [degree values (-90 - 90), distance from center (tool dependant)]

#tGripper - range of 275.00
tGripperRange = [400, 600]

#tVac - range of 335.00
tVacRange = [300, 600]

#Degree Values
degMin = -90.0
degMax = 90.0




def initRobot():
    # Primary Robot Interface
    R = abb.Robot(ip=robIP)

    R.set_speed(defaultSpeed)
    R.set_zone("z0", point_motion=True)

    R.set_joints(jZero)
    R.set_tool_advanced(tGripper_adv)
    R.set_Do_Vacume(0)
    R.set_Do_Grip(0)

    return R

def camera_alignment(R):
    '''Set's Robot to proper postion for aligning camera jig'''
    R.set_joints(jHome)
    R.set_tool()
    R.set_tool_advanced()
    R.set_cartesian([[625, 0.0, 235], [0.5, 0.5, 0.5, 0.5]])

def quatPlaneRotate(quat = [1, 0, 0, 0], axisIn = [0.0, 0.0, 1.0], deg = 0):
    q1 = Quaternion(quat)
    q2 = Quaternion(axis = axisIn, degrees=deg)

    q3 = q1 * q2

    return list(q3)
"""
def getRelativePos(pos, mod):
    newPos = pos[:]
    newPos[0] = newPos[0] + mod[0]
    newPos[1] = newPos[1] + mod[1]
    newPos[2] = newPos[2] + mod[2]

    return newPos

def getRelativeJoint(joint, mod):
    newJoint = joint[:]

    newJoint[0] = newJoint[0] + mod[0]
    newJoint[1] = newJoint[1] + mod[1]
    newJoint[2] = newJoint[2] + mod[2]
    newJoint[3] = newJoint[3] + mod[3]
    newJoint[4] = newJoint[4] + mod[4]
    newJoint[5] = newJoint[5] + mod[5]

    return newJoint
"""

def getRelativeVal(val, mod):
    newVal = val[:]
    for i in range(len(newVal)):
        newVal[i] += mod[i]

    return newVal

def getAdjustedVal(val, adj):
    newVal = val[:]
    for i in range(len(newVal)):
        if adj[i] != 0:
            newVal[i] = adj[i]

    return newVal

def getBaseAngle(loc = [200,200,0]):
    '''Input [X,Y,Z] location and return the angle necessary to align axis 1 with the object'''
    angle = 0
    if(loc[1] != 0):
        angle = math.degrees(math.atan(abs(loc[1])/abs(loc[0])))
        if(loc[1] < 0):
            angle = angle*(-1)


    return angle

def getRelativeJHome(R1, mod = [0,0,0]):
    R1.set_joints(jHome)
    cuCoord = R1.get_cartesian()

    R1.set_cartesian([getAdjustedVal(cuCoord[0],mod), cuCoord[1]])
    print(str(R1.get_cartesian()))
    return R1.get_cartesian()

def get_min_degree_seperation(stillObject):
    """
    Calculates and returns the angle of seperation necessary to avoid unwanted collisions between two move objects

    Parameters:
    stillObject: The object we are avoiding

    Returns:
    A tuple of two values representing the angle range to avoid
    """
    if stillObject.polarLoc[0] != 0:
        angle = math.degrees(math.atan((armWidth/2) / stillObject.polarLoc[0]))
    else:
        angle = 0
    return (stillObject.polarLoc[1] - angle, stillObject.polarLoc[1] + angle)

def random_value_with_exclusions(min_val, max_val, exclusions, decimal_places=2):
    """
    Generates a random value between min_val and max_val, excluding specified ranges.
    Exclusions should be a list of tuples, each tuple representing a range to exclude.
    """
    while True:
        value = random.uniform(min_val, max_val)
        value = round(value, decimal_places)
        if not any(lower <= value <= upper for lower, upper in exclusions):
            return value


def calculate_distance(coords_list1, coords_list2):
    """
    Calculates the Euclidean distance between two points represented by two lists of X and Y coordinates.

    Parameters:
    coords_list1: A list containing the X and Y coordinates of the first point [X1, Y1].
    coords_list2: A list containing the X and Y coordinates of the second point [X2, Y2].

    Returns:
    The Euclidean distance between the two points.
    """

    if len(coords_list1) != 2 or len(coords_list2) != 2:
        raise ValueError("Each list must contain exactly two elements representing X and Y coordinates.")

    x_diff = coords_list1[0] - coords_list2[0]
    y_diff = coords_list1[1] - coords_list2[1]

    distance = math.sqrt(x_diff ** 2 + y_diff ** 2)
    return distance

def polar_to_cartesian(r, theta):
    """
    Converts polar coordinates to Cartesian coordinates.

    Parameters:
    r: Radius (distance from the origin).
    theta: Angle in degrees from the X-axis (-90 to 90 degrees).

    Returns:
    A tuple (x, y) representing Cartesian coordinates.
    """

    # Convert theta from degrees to radians
    theta_rad = math.radians(theta)

    # Calculate x and y using the polar to Cartesian conversion formulas
    x = r * math.cos(theta_rad)
    y = r * math.sin(theta_rad)

    return [x, y]


def incrementalCartesianMove(R, cartesianLoc, stepNum):
    '''
    Cartesian moves are linear from one XYZ (and orient) to another.
    For full functionality, this would be broken into two parts for handling XYZ and Quaterions

    If we assume Quaterions will not change (IE, tool orientation will not change), then simply take the start pos (R.get_cartesian()) and
    draw a line to end pos (cartesianLoc). Then divide this line into 'stepNum' equal lenght segments.

    After each segment, the robot stops, allowing you to take photos
    In the case you want to keep moveObjs updated in real time, their data would also have to be updated after each segment before data collection.


    To add in Quaterions is a can of worms... I think it would be something like a hybrid joint movement with linear modification. IE, do something that resembles
    an incremental joint move (like below) while at the same time modifying the XYZ info in equal segments. If you attempt to implement this... good luck.
    '''
    
def incrementalJointMove(R, jointLoc, stepNum):
    '''
    joint movements happen Synchronously across all joints at once. To break a joint movement into a series of joint movements:
    get your starting joint position (R.get_joints())
    get your ending joint pos (jointLoc)
    FOR EACH JOINT INDIVIDUALLY: (end - start)/stepNum

    Note: I'm 97% confident this works this way... Needs testing.
    '''


def executeObjectRotation(R1, rotationObject, deg = 0.0):
    '''Executes robot movements to rotate movable objects'''
    liftHeight = 25
    dropHeight = 10

    degreeMod = 0
    #move robot to rotation work area
    #R1.set_joints(getRelativeVal(jHome,[-120,0,0,0,0,0]))
    if (-55 < rotationObject.polarLoc[1] < -45):
        R1.set_joints(getRelativeVal(jHome,[-120,0,0,0,0,0]))
    elif (45 < rotationObject.polarLoc[1] < 55):
        R1.set_joints(getRelativeVal(jHome,[120,0,0,0,0,0]))
    
    if abs(R1.get_joints()[0]) > 90:
        currentLoc = R1.get_cartesian()

        workCords = polar_to_cartesian(400, R1.get_joints()[0])
        workCords.append(rotationObject.cartLoc[0][2])
        workQuat = currentLoc[1]
    else:
        workCords = polar_to_cartesian(400, rotationObject.polarLoc[1])
        workCords.append(rotationObject.cartLoc[0][2])
        workQuat = rotationObject.cartLoc[1]
    
    
    workspaceLoc = [workCords, workQuat]

    R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, liftHeight]), workspaceLoc[1]])

    #Place rot object down
    #R1.set_cartesian(workspaceLoc)
    #rotationObject.activate_tool(0)
    #R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, liftHeight]), workspaceLoc[1]])


    #Determine rotation direction
    rotationDirection = 0
    if(rotationObject.rot <= deg):
        #clockwise rotation
        rotationDirection = 1
    else:
        #counterclockwise rotation
        rotationDirection = -1


    while(rotationObject.rot != deg):
        #repeate this loop until rotation of object is correct
        #turn at maximum 90 degree turns
        rotAmount = 0
        if(rotationDirection*(deg - rotationObject.rot) >= 90):
            #current degree is more than 90 degrees away, turn 90
            rotAmount = 90*rotationDirection
        else:
            rotAmount = deg - rotationObject.rot

        rotQuat = quatPlaneRotate(workspaceLoc[1], [0.0,0.0,1.0], rotAmount)



        #do pickup
        #R1.set_cartesian(workspaceLoc)
        #rotationObject.activate_tool(1)
        #R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, liftHeight]), workspaceLoc[1]])

        #rotate
        R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, liftHeight]), rotQuat])

        #place back
        R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, dropHeight]), rotQuat])
        rotationObject.activate_tool(0)
        R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, liftHeight]), rotQuat])
        R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, liftHeight]), workspaceLoc[1]])

        #update object rotation
        rotationObject.rot += rotAmount

        #If object is correctly rotated, the while loop will end. Otherwise, it continues
    #pickup object again for next move
    R1.set_cartesian([workspaceLoc[0], rotationObject.getApproachQuat(workspaceLoc[1])])
    rotationObject.activate_tool(1)
    R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, travelHeight]), workspaceLoc[1]])

def main():

    #Init robot
    R = initRobot()
    collector = DatasetCollector(total_photos=picCount, save_path='/Users/arm/Desktop/AutoDataCollection_Bottles/DataSet')
    #Init Metric Tracker
    metrics = metricTracker(save_path='/Users/arm/Desktop/AutoDataCollection_Bottles/DataSet', starting_iterations = collector.photo_count)

    #create mobile objects
    ketchupBottle = moveableObj(ketchupBottleStart, False, 0, [tGripper, tGripper_adv], tGripperRange, R.set_Do_Grip, defaultSpeed, bottleRadialEdge, bottleApproachAngle, bottleHeight)
    mustardBottle = moveableObj(mustardBottleStart, False, 0, [tGripper, tGripper_adv], tGripperRange, R.set_Do_Grip, defaultSpeed, bottleRadialEdge, bottleApproachAngle, bottleHeight)

    objectList = [ketchupBottle, mustardBottle]


    '''
    #MOVE OBJECT TEST
    #Example of a standard move Operation

    
    moveObj = mustardBottle
    
    #set object tool
    R.set_tool(moveObj.tool[0])
    moveObj.activate_tool(0)

    R.set_speed(moveSpeed)
    #Move to Angle
    R.set_joints(getRelativeVal(jHome, [moveObj.polarLoc[1], 0, 0, 0, 0, 0]))

    #move to above objects current location
                
    R.set_cartesian([getAdjustedVal(moveObj.cartLoc[0],[0,0,travelHeight]), moveObj.cartLoc[1]])
    R.set_speed(moveObj.maxLiftSpeed)
                
    #lower
    R.set_cartesian([moveObj.cartLoc[0], moveObj.getApproachQuat(moveObj.cartLoc[1])])
    #grab
    moveObj.activate_tool(1)
    #Raise
    R.set_cartesian([getAdjustedVal(moveObj.cartLoc[0], [0, 0, travelHeight]), moveObj.cartLoc[1]])

    #get "human location"
    degVal = 40
    R.set_joints(getAdjustedVal(jHome, [degVal, 0, 0, 0, 0, 0]))
    currQuat = R.get_cartesian()[1]
    humanPos = polar_to_cartesian(740, degVal)
    
    #move to above objects current location
                
    R.set_cartesian([[humanPos[0], humanPos[1], travelHeight], currQuat])
                
    #lower
    R.set_cartesian([[humanPos[0], humanPos[1], moveObj.height], currQuat])
    #grab
    moveObj.activate_tool(0)

    print([[humanPos[0], humanPos[1], moveObj.height], currQuat])
    exit()
    '''
    ##PROGRAM START
    
    '''
    General program flow:
    [START SECTION: No data recording, just move bottles to random starting locations]

    MAIN LOOP:
        For each bottle:
            preform incremental joint rotation to bottle rotation value
            preform incremental move to location above bottle
            incremental move down to bottle
            grab
            incremental raise

            preform incremental joint rotation to human location
            preform incremental move to location above human location
            incremental move down to humanLoc
            Drop
            incremental raise and move away??

            nonIncrent:
                grab bottle again
                move to new valid random position

            
            
    '''

    
    runBool = True

    while runBool:
        #Main Data Collection Loop - Runs once for every image in the dataset

        #In program data collection and time keeping
        metrics.start_timer()

        #determine object order
        if R.get_joints()[0] >= 0:
            sortReverse = True
        else:
            sortReverse = False
        sorted_objects = sorted(objectList, key=lambda obj: obj.jointLoc[0], reverse=sortReverse)

        
        #for each moving object
        for moveObj in sorted_objects:
            validPosition = False

            while not validPosition:
                ###### DETERMINE NEW RANDOM LOCATION ######
                #Pick random polar coord based on min and max tool values
                randDistance = random_value_with_exclusions(moveObj.toolMinMax[0], moveObj.toolMinMax[1], [], 3)

                #### Pick random polar angle while avoiding potential trouble angles ####

                excludedRanges = []

                #For every other movable object, determine problem angle ranges and add them to list
                for otherObj in objectList:
                    # for every object in the object list, determine all excluded ranges
                    if otherObj != moveObj:
                        excludedRanges.append(get_min_degree_seperation(otherObj))

                randAngle = random_value_with_exclusions(-80.0, 80.0, excludedRanges, 2)

                #convert Polar Coords to X,Y Coords

                randCartesianCoord = polar_to_cartesian(randDistance, randAngle)

                #determine if coord is the minimum distance from all other movable objects
                
                validPosition = True

                #check for conditionals
                if (metrics.run_count % 10 == 0) or (minMoveDistance <= (calculate_distance(randCartesianCoord, [moveObj.cartLoc[0][0], moveObj.cartLoc[0][1]])) <= maxMoveDistance):
                    if (-330 < randCartesianCoord[1] < -270) or (270 < randCartesianCoord[1] < 330):
                        validPosition = False
                    else:
                        for otherObj in objectList:
                            # for every object in the object list
                            if otherObj != moveObj:
                                seperationDistance = calculate_distance(randCartesianCoord, [otherObj.cartLoc[0][0], otherObj.cartLoc[0][1]])
                                if seperationDistance < (minSeperation+otherObj.radialEdge+moveObj.radialEdge):
                                    validPosition = False
                                    break
                else:
                    validPosition = False
            
            #With a valid set of coordinates...

            #set object tool
            R.set_tool(moveObj.tool[0])
            moveObj.activate_tool(0)

            R.set_speed(moveSpeed)
            #Move to Angle
            R.set_joints(getRelativeVal(jHome, [moveObj.polarLoc[1], 0, 0, 0, 0, 0]))

            #move to above objects current location
            
            R.set_cartesian([getAdjustedVal(moveObj.cartLoc[0],[0,0,travelHeight]), moveObj.cartLoc[1]])
            R.set_speed(moveObj.maxLiftSpeed)
            
            #lower
            R.set_cartesian([moveObj.cartLoc[0], moveObj.getApproachQuat(moveObj.cartLoc[1])])
            #grab
            moveObj.activate_tool(1)
            #Raise
            R.set_cartesian([getAdjustedVal(moveObj.cartLoc[0], [0, 0, travelHeight]), moveObj.cartLoc[1]])
            
            #If object has rotation variety, pick a random rotation from 0-359.99
            randRot = 0
            if moveObj.isRot:
                randRot = random_value_with_exclusions(0.0, 90.0, [], 2)
                executeObjectRotation(R, moveObj, randRot)

            
            R.set_speed(moveSpeed)
            #Rotate joint 1 to new position
            R.set_joints(getRelativeVal(jHome, [randAngle, 0, 0, 0, 0, 0]))
            
            #get updated cartesian rotation data
            newQuat = R.get_cartesian()[1]
            #above
            R.set_cartesian([[randCartesianCoord[0],randCartesianCoord[1], travelHeight], newQuat])
            R.set_speed(defaultSpeed)
            
            moveObj.zMod = 0
            #accounting for slanting table
            if randCartesianCoord[1] <= -480:
                    moveObj.zMod = -2
            elif randCartesianCoord[1] >= 320:
                moveObj.zMod = 2

            

            #lower
            R.set_cartesian([[randCartesianCoord[0],randCartesianCoord[1], (moveObj.height + moveObj.zMod)], moveObj.getApproachQuat(newQuat)])

            #Update Object Data Values for: cartLoc, polarLoc, jointLoc, rot

            moveObj.cartLoc = [[randCartesianCoord[0],randCartesianCoord[1], (moveObj.height + moveObj.zMod)], newQuat]
            moveObj.polarLoc = [randDistance, randAngle]
            moveObj.jointLoc = R.get_joints()
            moveObj.rot = randRot

            #drop
            moveObj.activate_tool(0)
            # Raise
            R.set_cartesian([getAdjustedVal(moveObj.cartLoc[0], [0, 0, travelHeight]), moveObj.cartLoc[1]])

        ##All Move Objects have new positions##

        #Move Robot out of the way
        R.set_speed(moveSpeed)
        if R.get_joints()[0] >= 0:
            R.set_joints(jOffScreen)
        else:
            R.set_joints(jOffScreenNeg)

        #Record Data
        photo_name = collector.capture_photo()
        collector.update_csv(photo_name, objectList)

        #Lap Data Collection
        metrics.end_timer()
        metrics.output_to_csv()

        if (metrics.run_count % 15 == 0):
            time.sleep(40)

        print(f"==== {collector.photo_count} / {collector.total_photos} ====")
        print(f"Streak  : {metrics.run_count}")
        print(f"Average Time  : {metrics.get_average_time()}")
        print(f"Previous Loop : {metrics.loop_time}")
        print(f"Estimated Remaining Time: {metrics.format_time(metrics.estimate_remaining_time(picCount))}")
        #Continue to next image...

    R.set_joints(jZero)
    R.close()
    collector.close_class()
    


main()

