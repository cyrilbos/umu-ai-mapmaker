#!/usr/bin/env python3
"""
A robot controller for MRDS.

Follows a given path using an implementation of the pure pursuit
algorithm.

Authors: Ville Gillström (oi14vgm@cs.umu.se)
         Tobias Nebaeus (c14tns@cs.umu.se)

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
    Output: The angle between pose and nextPosition
    """
    
    currentPosition = pose['Pose']['Position']
    dy = nextPosition['Y'] - currentPosition['Y'] 
    dx = nextPosition['X'] - currentPosition['X']
    currentBearing = heading(pose['Pose']['Orientation'])
    
    robotAngle = atan2(currentBearing['Y'], currentBearing['X'])
    angleToPosition = atan2(dy, dx)
    angle = robotAngle - angleToPosition

    # fixes weird angle bug
    while angle > pi:
        angle =-  pi
    while angle < -pi:
        angle =+  pi
    
    return angle



def getNextCarrotNode(pose, curNodeNum, path):
    '''
    Get the index of the last node on the path which is within lookahead distance,
    starting from index curNodeNum
    Input:  pose - Robot's current pose
            curNodeNum - Index of the current goal node
            path - The path to follow
    Output: The carrot (goal) node
    '''

    while (isWithinLOOKAHEAD(pose['Pose']['Position'],
            path[curNodeNum]['Pose']['Position']) and
            (curNodeNum < len(path) - 1)):
        curNodeNum += 1;
    
    return curNodeNum

def getNextCarrotPosition(pose, curNodeNum, path):
    '''
    Get the exact position of the carrot point on the path segment
    between node curNodeNum and the next one
    Input:  pose - Robot's current pose
            curNodeNum - Index of the current goal node
            path - The path to follow
    Output: The carrot (goal) position
    '''

    if curNodeNum == len(path) - 1:
        return path[curNodeNum]['Pose']['Position']

    x1 = path[curNodeNum]['Pose']['Position']['X']
    x2 = path[curNodeNum + 1]['Pose']['Position']['X']
    y1 = path[curNodeNum]['Pose']['Position']['Y']
    y2 = path[curNodeNum + 1]['Pose']['Position']['Y']

    distanceToAdd = LOOKAHEAD - getDistance(pose['Pose']['Position'], 
                                path[curNodeNum]['Pose']['Position'])

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
    '''
    Returns the necessary angular speed to follow a generated circle
    segment between the robot and goal point.
    Warning: the speed might be higher than the maximum
    angular speed of the robot.
    Input:  pose - Robot's current pose
            goalPosition - The current goal position
            linearSpeed - The given linear speed
    Output: The angular speed
    '''

    L = LOOKAHEAD
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
       
        linearSpeed = 1.0
        angularSpeed = getPureAngularSpeed(pose, nextPose['Position'], linearSpeed)
       
        #Slow down when doing sharp turns
        while abs(angularSpeed) > 1.7: # värdet behöver optimeras
            linearSpeed -= 0.01
            angularSpeed = getPureAngularSpeed(pose, nextPose['Position'], linearSpeed)
        
        linearSpeed -= 0.1

        postSpeed(angularSpeed, linearSpeed)
        time.sleep(0.02)

        pose = getPose()
        LOOKAHEAD = 0.90
        curNodeNum = getNextCarrotNode(pose, curNodeNum, path)
        newCarrotPosition = getNextCarrotPosition(pose, curNodeNum, path)

        # "om avståndet är kort och vinkeln hög: minska lookahead och fixa ny carrot point"
        while (LOOKAHEAD >= 0.3 and (abs(getAngle(pose, newCarrotPosition) /
                getDistance(pose['Pose']['Position'], newCarrotPosition) > 0.20))):
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
    