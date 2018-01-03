#!/usr/bin/env python3
"""
Example demonstrating how to communicate with Microsoft Robotic Developer
Studio 4 via the Lokarria http interface. 

Author: Erik Billing (billing@cs.umu.se)

Updated by Ola Ringdahl 2014-09-11
Updated by Lennart Jern 2016-09-06 (converted to Python 3)
Updated by Filip Allberg and Daniel Harr 2017-08-30 (actually converted to Python 3)
"""

from src.mapper.util import *

LOOKAHEAD = 0.95
GOAL_THRESHOLD = 1.0



def isWithinLOOKAHEAD(position, nextPosition):
    """
     Checks wether the distance between two positions is less than LOOKAHEAD
     Input: position - A postition, usually the robots current position
     Input: nextPosition - A postition
     Output: Bool - True if the distance between two positions is less than LOOKAHEAD, else false.
    """

    # currentPosition = pose['Pose']['Position']
    #if __debug__:
    #    print('X:%.4f, Y:%.4f ' %(currentPosition['X'], currentPosition['Y']))

    return getDistance(position, nextPosition) <= LOOKAHEAD
    

def getDistance(position1, position2):
    """
     Get distance between to positions
     Input: position1 - A postition
     Input: position2 - A postition
     Output: The distance between two positions
    """
    dy = abs(position1['Y'] - position2['Y'])
    dx = abs(position1['X'] - position2['X'])

    return hypot(dy, dx);
    
def getAngle(pose, nextPosition):
    """
    Get angle between robots heading and nextPosition
    Input:  pose - Robot's current pose
            nextPosition - Position to get angle to
    """
    currentPosition = pose['Pose']['Position']
    dy = nextPosition['Y'] - currentPosition['Y'] 
    dx = nextPosition['X'] - currentPosition['X']
    currentBearing = heading(pose['Pose']['Orientation'])
    
    robotAngle = atan2(currentBearing['Y'], currentBearing['X'])
    
    angleToPosition = atan2(dy, dx)
    
    angle = robotAngle - angleToPosition
    
    return angle

def findClosestPositionOnPath(pose, curNodeNum, path):
    # https://stackoverflow.com/questions/3120357/get-closest-point-to-a-line
    if curNodeNum == 0 :
        return path[curNodeNum]['Pose']['Position']

    i = curNodeNum
    while (getDistance(pose['Pose']['Position'],
            path[i]['Pose']['Position']) >
            getDistance(pose['Pose']['Position'],
            path[i + 1]['Pose']['Position'])):
        i += 1


    x1 = path[i]['Pose']['Position']['X']
    y1 = path[i]['Pose']['Position']['Y']
    x2 = path[i + 1]['Pose']['Position']['X']
    y2 = path[i + 1]['Pose']['Position']['Y']
    px = pose['Pose']['Position']['X']
    py = pose['Pose']['Position']['Y']
    
    a_to_p = (px - x1, py - y1)
    a_to_b = (x2 - x1, y2 - y1)

    atb2 = a_to_b[0]**2 + a_to_b[1]**2

    atp_dot_atb = a_to_p[0]*a_to_b[0] + a_to_p[1]*a_to_b[1]
                                    
    t = atp_dot_atb / atb2 if atb2 != 0 else atp_dot_atb

    position = {
        'X' : (x1 + a_to_b[0]*t),
        'Y' : (y1 + a_to_b[1]*t)
        }

    return position


def getNextCarrotNode(pose, curNodeNum, path):
    #closestPositionOnPath = findClosestPositionOnPath(pose, curNodeNum, path)
    #while isWithinLOOKAHEAD(closestPositionOnPath, path[curNodeNum]['Pose']['Position']) and (curNodeNum < len(path) - 1):
    #    curNodeNum += 1;

    while (isWithinLOOKAHEAD(pose['Pose']['Position'], path[curNodeNum]['Pose']['Position']) 
        and (curNodeNum < len(path) - 1)):
        curNodeNum += 1;

    print ("NodeNr: " + str(curNodeNum))
    
    return curNodeNum

def getNextCarrotPosition(pose, curNodeNum, path):
    if curNodeNum == len(path) - 1:
        return path[curNodeNum]['Pose']['Position']

    x1 = path[curNodeNum]['Pose']['Position']['X']
    x2 = path[curNodeNum + 1]['Pose']['Position']['X']
    y1 = path[curNodeNum]['Pose']['Position']['Y']
    y2 = path[curNodeNum + 1]['Pose']['Position']['Y']

    distanceToAdd = LOOKAHEAD - getDistance(pose['Pose']['Position'], 
                                path[curNodeNum]['Pose']['Position'])

    #closestPositionOnPath = findClosestPositionOnPath(pose, curNodeNum, path)
    #distanceToAdd = LOOKAHEAD - getDistance(closestPositionOnPath,
    #                path[curNodeNum]['Pose']['Position'])


    dy = y2 - y1;
    dx = x2 - x1;
    if dx == 0:
        xToAdd = 0
        yToAdd = distanceToAdd
    else:
        dydx = dy / dx;
        xToAdd = sqrt((distanceToAdd ** 2) / (1 + dydx ** 2))
        yToAdd = dydx * xToAdd

    newX = x1 + xToAdd
    newY = y1 + yToAdd

    carrotPosition = { 'X' : newX, 'Y' : newY }

    return carrotPosition



def getPureAngularSpeed(pose, goalPosition, linearSpeed):
    # TODO L stämmer inte för sista noden!
    L = LOOKAHEAD # ta som param senare (ej konstant)
    y = sin(getAngle(pose, goalPosition)) * L
    r = (L ** 2) / (2 * y)
    Y = 1 / r

    angularVelocity = linearSpeed * Y

    return - angularVelocity



def purePursuit(path):
    """
    Follow the path using pure pursuit
    Input: path - filepath to the file containg a path
    """
    pose = getPose()
    startNodeNum = 0
    curNodeNum = getNextCarrotNode(pose, startNodeNum, path)

    nextPose = path[curNodeNum]['Pose']


    while not ((curNodeNum == len(path) - 1) and
            (getDistance(pose['Pose']['Position'], path[-1]['Pose']['Position']) < 1.0)):
       
        linearSpeed = 2.0
        angularSpeed = getPureAngularSpeed(pose, nextPose['Position'], linearSpeed)
       
        #Slow down when doing sharp turns
        while abs(angularSpeed) > 1.7: # värdet behöver optimeras
            linearSpeed -= 0.01
            angularSpeed = getPureAngularSpeed(pose, nextPose['Position'], linearSpeed)

        postSpeed(angularSpeed, linearSpeed)
        time.sleep(0.02)

        pose = getPose()
        LOOKAHEAD = 0.95
        curNodeNum = getNextCarrotNode(pose, curNodeNum, path)
        newCarrotPosition = getNextCarrotPosition(pose, curNodeNum, path)

        # "om avståndet är kort och vinkeln hög: minska lookahead och fixa ny carrot point"
        while (LOOKAHEAD >= 0.3 and (abs(getAngle(pose, newCarrotPosition) /
                getDistance(pose['Pose']['Position'], newCarrotPosition) > 0.25))): # värdet behöver optimeras
            LOOKAHEAD -= 0.05
            curNodeNum = getNextCarrotNode(pose, curNodeNum, path)
            newCarrotPosition = getNextCarrotPosition(pose, curNodeNum, path)

        nextPose = { 'Position' : newCarrotPosition }

    postSpeed(0, 0)

    
###############################################################################

    
if __name__ == '__main__':
    if len(sys.argv) > 1:
        filename = str(sys.argv[1]) 
        print ('Path file:', filename)
        #Assumes all arguments are paths
        pathFiles = sys.argv[1:]
    else:
        pathFiles = [
            "paths\\Path-to-bed.json",
            "paths\\Path-from-bed.json",
            "paths\\Path-around-bench-and-sofa.json",
            "paths\\Path-around-table-and-sofa.json",
            "paths\\Path-around-table.json"
            ]


    for p in pathFiles:
        #Open path file
        with open(p) as path_file:    
            path = json.load(path_file)

        start_time = time.time();
        purePursuit(path)
        end_time = time.time() - start_time
        print("Time for " + p + ": " + str(end_time))
        time.sleep(1)


    print("DONE")
    
    
    


def followTheCarrot(path):
    currentPosition = getPose()
    
    startNodeNum = 0
    curNodeNum = getNextCarrotNode(startNodeNum, path)
    
    #print('X:%.4f, Y:%.4f ' %(path[curNodeNum]['Pose']['Position']['X'], path[curNodeNum]['Pose']['Position']['Y']))
    nextPose = path[curNodeNum]['Pose']
    #print (nextPose)
    
    angle = getAngle(nextPose['Position'])

    while True:
        angle = getAngle(nextPose['Position'])
        if abs(angle) > 0.10:
            linearSpeed = 0.2
            angularSpeed = -2*(2 / pi) * angle
            #angularSpeed = getPureAngularSpeed(nextPose['Position'], linearSpeed)
        else:
            angularSpeed = 0
            linearSpeed = 0.8
                
        postSpeed(angularSpeed, linearSpeed)
        time.sleep(0.05)
        curNodeNum = getNextCarrotNode(curNodeNum, path)
        newCarrotPosition = getNextCarrotPosition(curNodeNum, path)
        nextPose = { 'Position' : newCarrotPosition }

    
    
    
    # print('Sending commands to MRDS server', MRDS_URL)
    # try:
        # print('Telling the robot to go streight ahead.')
        # response = postSpeed(0,0.5) 
        # print('Waiting for a while...')
        # time.sleep(3)
        # print('Telling the robot to go in a circle.')
        # response = postSpeed(0.9,0.1)        
    # except UnexpectedResponse as ex:
        # print('Unexpected response from server when sending speed commands:', ex)

    # try:
        # laser = getLaser()
        # laserAngles = getLaserAngles()
        # print('The rightmost laser bean has angle %.3f deg from x-axis (streight forward) and distance %.3f meters.'%(
            # laserAngles[0],laser['Echoes'][0]
        # ))
        # print('Beam 1: %.3f Beam 269: %.3f Beam 270: %.3f'%( laserAngles[0]*180/pi, laserAngles[269]*180/pi, laserAngles[270]*180/pi))
    # except UnexpectedResponse as ex:
        # print('Unexpected response from server when reading laser path:', ex)


    # try:
        # pose = getPose()
        # print('Current position: ', pose['Pose']['Position'])
        # for t in range(30):    
            # print('Current heading vector: X:{X:.3}, Y:{Y:.3}'.format(**getBearing()))
            # laser = getLaser()
            # print('Distance %.3f meters.'%(laser['Echoes'][135]))
            # if (laser['Echoes'][135] < 0.3):
                # print('Danger! Brace for impact! Hit the brakes!')
                # response = postSpeed(0,-0.1)
            # time.sleep(1)
    # except UnexpectedResponse as ex:
        # print('Unexpected response from server when reading position:', ex)
