import abb
from pyquaternion import Quaternion
import math, time, random, cv2, ast, sys, numpy, os
from threading import Thread



class moveableObj:
    """A class for movable objects in our workspace. Tracks X,Y,Z position and rotation of objects, and the tool they are used to interact with"""
    def __init__(self, initLoc, initToolAngle, initIsRot, initRot, toolData, tool_command):

        #Tracks the current location of the cylinder in the world plane as [X, Y, Z]
        self.loc = initLoc

        #Tracks current rotation of object using quaternion of tool [q1,q2,q3,q4]
        self.toolAngle = initToolAngle

        #Bool value that determines if object should have random rotational values
        self.isRot = initIsRot

        #relative to the world, the degree value of rotation with respect to the XY plane
        self.rot = initRot

        #Establishes the tool used to interact with object
        self.tool = toolData

        #Refers to the Set_Do command associated with the tool
        self.set_Tool = tool_command

robIP = '192.168.125.1'

defaultSpeed = [100, 500, 5000, 1000]
squareLiftSpeed = [300,500,5000,1000]
defaultZone = [0,0,0]

#TOOLS
tGripper = [[0,13.71,142.91],[0.5,-0.5,-0.5,-0.5]]
tGripper_adv = [.5,[[2,0,61.2],[1,0,0,0]]]

tVac = [[0,-44.72,35.37],[0.5,-0.5,-0.5,-0.5]]
#tVac = [[0, -46.72, 35.37], [0.37992819659091526, -0.5963678105290181, -0.5963678105290181, -0.37992819659091526]]
tVac_adv = [.5,[[2,0,61.2],[1,0,0,0]]]

tVacRot = [[0, -46.72, 35.37], [0.4304593345768794, -0.560985526796931, -0.560985526796931, -0.4304593345768794]]

#Manual Joint rotations
jHome = [0,45,45,0,-90,90]
jZero = [0,0,0,0,0,0]

camCenter = [[452.0, -5.0, 585.1], [0.5, 0.5, 0.5, 0.5]]

#Program Parameters
picCount = 100


#WorkObjectTest

wTable = [[0,0,0],[.999993,0.00333701,0.00171519,0]]

#Placeholders, need to set these based on physical location
cylinderStartPos = [[-337.2, 581.91, 100.0], [0.5, 0.0, 0.0, 0.866]]
squareStartPos = [[-179.9, 308.39, 0], [0.5,0.0,0.0,0.866]]

#Random number generator parameters
#Determined by tool size and reachable position relative to the table.
#ROTATIONAL VALUES: [degree values (-90 - 90), distance from center (tool dependant)]

#tGripper
tGripperMin = 400
tGripperMax = 610

#tVac
tVacMin = 300
tVacMax = 600

def quatPlaneRotate(quat = [1, 0, 0, 0], axisIn = [0.0, 0.0, 1.0], deg = 0):
    q1 = Quaternion(quat)
    q2 = Quaternion(axis = axisIn, degrees=deg)

    q3 = q1 * q2

    return list(q3)

def getRelativeJHome(R1, mod = [0,0,0]):
    R1.set_joints(jHome)
    cuCoord = R1.get_cartesian()

    R1.set_cartesian([getAdjustedVal(cuCoord[0],mod), cuCoord[1]])
    print(str(R1.get_cartesian()))
    return R1.get_cartesian()


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



def initRobot():
    # Primary Robot Interface
    R = abb.Robot(ip=robIP)

    R.set_speed(defaultSpeed)
    R.set_zone(point_motion=True)

    R.set_tool_advanced(tGripper_adv)

    return R



def getBaseAngle(loc = [200,200,0]):
    '''Input [X,Y,Z] location and return the angle necessary to align axis 1 with the object'''
    angle = 0
    if(loc[1] != 0):
        angle = math.degrees(math.atan(abs(loc[1])/abs(loc[0])))
        if(loc[1] < 0):
            angle = angle*(-1)


    return angle

def executeObjectRotation(R1, rotationObject, deg = 0.0):
    '''Executes robot movements to rotate movable objects'''

    #move robot to rotation work area
    R1.set_joints(getRelativeJoint(jHome,[-120,0,0,0,0,0]))
    currentLoc = R1.get_cartesian()

    workspaceLoc = [[currentLoc[0][0], currentLoc[0][1], 0], currentLoc[1]]

    #Place rot object down
    R1.set_cartesian(workspaceLoc)
    R1.setDoVacume(0)
    R1.set_cartesian([getRelativePos(workspaceLoc[0], [0, 0, 50]), workspaceLoc[1]])


    #Determine rotation direction
    rotationDirection = 0
    if(rotationObject.rot <= deg):
        #clockwise rotation
        rotationDirection = 1
    else:
        #counterclockwise rotation
        rotationDirection = -1

    print("Initial rotationObject.rot: {}".format(rotationObject.rot))
    print("Target deg: {}".format(deg))
    print("Initial rotation direction: {}".format("Clockwise" if rotationDirection == 1 else "Counterclockwise"))


    while(rotationObject.rot != deg):
        #repeate this loop until rotation of object is correct
        #turn at maximum 90 degree turns
        rotAmount = 0
        if(rotationDirection*(deg - rotationObject.rot) >= 90):
            #current degree is more than 90 degrees away, turn 90
            rotAmount = 90*rotationDirection
        else:
            rotAmount = deg - rotationObject.rot

        rotQuat = quatPlaneRotate(workspaceLoc[1], rotAmount)



        #do pickup
        R1.set_cartesian(workspaceLoc)
        R1.setDoVacume(1)
        R1.set_cartesian([getRelativePos(workspaceLoc[0], [0, 0, 50]), workspaceLoc[1]])

        #rotate
        R1.set_cartesian([getRelativePos(workspaceLoc[0], [0, 0, 50]), rotQuat])

        #place back
        R1.set_cartesian([getRelativePos(workspaceLoc[0], [0, 0, 0]), rotQuat])
        R1.setDoVacume(0)
        R1.set_cartesian([getRelativePos(workspaceLoc[0], [0, 0, 50]), rotQuat])
        R1.set_cartesian([getRelativePos(workspaceLoc[0], [0, 0, 50]), workspaceLoc[1]])

        #update object rotation
        rotationObject.rot += rotAmount

        print("Updated rotationObject.rot: {}".format(rotationObject.rot))
        print("Remaining rotation to target deg: {}".format(deg - rotationObject.rot))
        #If object is correctly rotated, the while loop will end. Otherwise, it continues

    #pickup object again for next move
    R1.set_cartesian(workspaceLoc)
    R1.set_Do_Vacume(1)
    R1.set_cartesian([getRelativePos(workspaceLoc[0], [0, 0, 50]), workspaceLoc[1]])



def main():
    #Init robot
    R = initRobot()
    '''
    R.set_joints(jHome)
    #R.set_joints(getRelativeJoint(jHome,[127.5,0,0,0,0,0]))
    R.set_tool(tGripper)
    jHomeCart = R.get_cartesian()
    print(jHomeCart)
    R.set_cartesian([getAdjustedVal(jHomeCart[0],[0,0,325]), jHomeCart[1]])
    print(R.get_joints())
    '''
    #print(R.get_cartesian())
    #R.set_cartesian([[-428, 581.91, 270], [0.442, 0.0, 0.0, 0.897]])
    #R.set_cartesian([[-428, 581.91, 70], [0.442, 0.0, 0.0, 0.897]])
    #R.set_Do_Grip(0)
    #R.set_cartesian([[-428, 581.91, 270], [0.442, 0.0, 0.0, 0.897]])
    #R.set_tool(tVac)
    #R.set_speed(squareLiftSpeed)
    #dataset = '/Users/arm/Desktop/DataSet/dataset.csv'

    #squarePoint = extract_data(dataset, 23, 18)
    tools_off(R)
    #checkPos = [[518.2529415092389,249.6411397226456,0],[0.975,0.0,0.0,0.223]]
    #checkPosZ = [[518.2529415092389,249.6411397226456,50],[0.975,0.0,0.0,0.223]]
    #R.set_tool(tVac)
    #R.set_speed([300,500,5000,1000])

    #R.set_cartesian(checkPosZ)
    #R.set_cartesian([checkPos[0], quatPlaneRotate(checkPos[1],[0.0,1.0,0.0], 10)])
    #R.set_Do_Vacume(1)
    #R.set_cartesian(checkPosZ)
    #camera_alignment(R)
    #R.set_cartesian([[444.0, -4.9, 578.8], [0.5, 0.5, 0.5, 0.5]])
    #camera_position_detection_test(R)
    #get_camera_centered_set(R)
    R.close()

def tools_off(R):
    R.set_Do_Grip(0)
    R.set_Do_Vacume(0)

def defineWorkObj():
    R = initRobot()
    R.set_joints(jHome)
    R.set_tool(tVac)

    x1 = [400, -200, 200]
    x2 = [500, -200, 200]
    y1 = [200,500,200]
    points = [x1,x2,y1]

    for pos in points:
        moveRobToLoc(R, pos)
        inp = input("Press any key to continue")


def getTableSpread():
    R = initRobot()
    R.set_joints(jHome)
    R.set_tool(tVac)

    

def getApproachQuat(modQuat):
        if self.approachAngle == 0:
                return modQuat
        else:
            q1 = Quaternion(modQuat)
            q2 = Quaternion(axis = [0.0,1.0,0.0], degrees = 10)

            q3 = q1 * q2

            return list(q3)
        
def extract_data(csv_file, line_number, start_column):
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        for i, row in enumerate(reader):
            if i == line_number:
                # Extract x, y, z, q1, q2, q3, q4 from the row starting from start_column
                x, y, z = row[start_column:start_column + 3]
                q1, q2, q3, q4 = row[start_column + 3:start_column + 7]
                return [[x, y, z], [q1, q2, q3, q4]]

    return None  # Return None if the line number is not found



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

def printCurrentLoc():
    R = initRobot()
    R.set_tool(tVac)
    R.set_tool_advanced(tVac_adv)

    print(R.get_cartesian())

    R.close()

def findToolEnds():
    R = initRobot()
    R.set_joints(jHome)

    R.set_tool(tVac)
    R.set_tool_advanced(tGripper_adv)

    currentCart = R.get_cartesian()
    print("jHome Vac Position: "+str(currentCart))

    #positiveDir

    R.set_cartesian([[currentCart[0][0],currentCart[0][1],0], currentCart[1]])

    currentCart = R.get_cartesian()

    for x in range(int(currentCart[0][0]), 000, -1):
        R.set_cartesian([[x,currentCart[0][1],0], currentCart[1]])
        print(x)

    return R

def confirmToolEnds():
    R = initRobot()
    '''
    toolList = [tGripper, tGripper, tVac, tVac]
    valueList = [tGripperMin, tGripperMax, tVacMin, tVacMax]
    heightList = [75, 75, 2, 2]
    '''
    #R.set_workobject(wTable)
    
    toolList = [tVac, tVac]
    valueList = [tVacMin, tVacMax]
    heightList = [2, 2]
    
    masterList = [toolList, valueList]

    for x in range(len(masterList[0])):
        R.set_joints(jHome)
        R.set_tool()
        R.set_tool(toolList[x])

        print(R.get_tool())

        cuCar = R.get_cartesian()

        R.set_cartesian([[valueList[x], cuCar[0][1], heightList[x]], quatPlaneRotate(cuCar[1], [0.0,1.0,0.0], 15)])

        jVal = R.get_joints()

        for deg in range(0, -90, -5):
            R.set_joints(getRelativeJoint(jVal, [deg, 0, 0, 0, 0, 0]))
        for deg in range(-90, 90, 5):
            R.set_joints(getRelativeJoint(jVal, [deg, 0, 0, 0, 0, 0]))
        for deg in range(90, 0, -5):
            R.set_joints(getRelativeJoint(jVal, [deg, 0, 0, 0, 0, 0]))

def getBaseAngle(loc):
    '''Input [X,Y,Z] location and return the angle necessary to align axis 1 with the object'''
    angle = 0
    if(loc[1] != 0):
        angle = math.degrees(math.atan(abs(loc[1])/abs(loc[0])))
        if(loc[1] < 0):
            angle = angle*(-1)


    return angle

def moveRobToLoc(R, loc):
    rotAngle = getBaseAngle(loc)
    R.set_joints(getAdjustedVal(R.get_joints(), [rotAngle,0,0,0,0,0]))
    cart = R.get_cartesian()
    quat = cart[1]

    R.set_cartesian([loc, quat])
    
    
def jogL(R):
    print("Jogging Robot... enter [0,0,0] to end jog")
    isJogging = True
    while isJogging:
        currentPos = R.get_cartesian()

        entry = input("Enter Coords: ")
        entry = ast.literal_eval(entry)

        if(entry == [0,0,0]):
            isJogging = False
        
        R.set_cartesian([[currentPos[0][0]+entry[0],currentPos[0][1]+entry[1],currentPos[0][2]+entry[2]],currentPos[1]])
        time.sleep(.1)
    print(R.get_cartesian())
    return R.get_cartesian()


def camera_alignment(R):
    '''Set's Robot to proper postion for aligning camera jig'''
    R.set_joints(jHome)
    R.set_tool()
    R.set_tool_advanced()
    R.set_cartesian([[625, 0.0, 235], [0.5, 0.5, 0.5, 0.5]])

def camera_output_test():
    cap = cv2.VideoCapture(0)
    lineSize = 10
    frameWidth = 3840
    frameHeight = 2160
    
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)

    (ret, frame) = cap.read()
    print('Resolution: ' + str(frame.shape[0]) + ' x ' + str(frame.shape[1]))
    cv2.imwrite('/Users/arm/Desktop/CamCalib.jpg', frame)

def stream_video_w_overlay():
    cap = cv2.VideoCapture(0)
    window_name = 'image'
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    lineSize = 2
    frameWidth = 3840
    frameHeight = 2160

    wDiv2 = frameWidth/2
    hDiv2 = frameHeight/2

    print([frameWidth, frameHeight, wDiv2, hDiv2])
    
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)

    while True:
        #get image
        (_, img) = cap.read()

        #draw cross lines
        cv2.line(img, (int(wDiv2), int(0)), (int(wDiv2),int(frameHeight)), (255,255,255), lineSize)
        cv2.line(img, (int(0), int(hDiv2)), (int(frameWidth), int(hDiv2)), (255,255,255), lineSize)
        
        #show
        cv2.imshow(window_name, img)
        if cv2.waitKey(10) & 0xff ==ord('q'):
            break

    cv2.destroyWindow(window_name)
    

def camera_position_detection_test(R):
    
    t1 = Thread(target = jogL, args=(R,),)

    t1.start()
    stream_video_w_overlay()
    print(t1.join())

def capture_centered_set(name):
    cap = cv2.VideoCapture(0)
    lineSize = 2
    frameWidth = 1920
    frameHeight = 1080

    wDiv2 = frameWidth/2
    hDiv2 = frameHeight/2

    #print([frameWidth, frameHeight, wDiv2, hDiv2])
    
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)

    
    #get image
    (_, img) = cap.read()
    cv2.imwrite(str(name)+ ".jpg", img)
            
    #draw cross lines
    cv2.line(img, (int(wDiv2), int(0)), (int(wDiv2),int(frameHeight)), (255,255,255), lineSize)
    cv2.line(img, (int(0), int(hDiv2)), (int(frameWidth), int(hDiv2)), (255,255,255), lineSize)
        
    cv2.imwrite(str(name)+ "_lines.jpg", img)
    
    cap.release()

    
    

def get_camera_centered_set(R):
    startPoint = R.get_cartesian()
    
    for i in range(0, 310, 10):
        capture_centered_set(str(i))
        R.set_cartesian([[startPoint[0][0],startPoint[0][1],startPoint[0][2]-i],startPoint[1]])
        time.sleep(1)
main()
#confirmToolEnds()
#capture_centered_set("test")
