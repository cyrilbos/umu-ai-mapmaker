#!/usr/bin/env python3
"""
A robot controller for MRDS.

Follows a given path using an implementation of the pure pursuit
algorithm.

Authors: Ville Gillström (oi14vgm@cs.umu.se)
         Tobias Nebaeus (c14tns@cs.umu.se)

"""
from logging import getLogger
from math import atan
from multiprocessing import Queue

from .util import *

MAX_LOOKAHEAD = 1.4
MIN_LOOKAHEAD = 0.5
GOAL_THRESHOLD = 1.0
ROBOT_WIDTH = 0.9 # actual is 0.4

logger = getLogger(__name__)

g_lookahead = 1.0
def isWithinLOOKAHEAD(position, nextPosition):
    """
     Checks whether the distance between two positions is less than LOOKAHEAD
     Input: position - A postition, usually the robots current position
     Input: nextPosition - A postition
     Output: Bool - True if the distance between two positions is less than LOOKAHEAD, else false.
    """

    return getDistance(position, nextPosition) <= g_lookahead
    

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

    # Fixes weird angle bug
    while angle > pi:
        angle =-  pi
    while angle < -pi:
        angle =+  pi
    
    return angle


def getNextCarrotNode(pose, curNodeNum, path, lsr, lsrAngles):
    '''
    Get the next node to "aim" at
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

    distanceToAdd = g_lookahead - getDistance(pose['Pose']['Position'], 
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

    L = g_lookahead
    y = sin(getAngle(pose, goalPosition)) * L
    r = (L ** 2) / (2 * y)
    Y = 1 / r

    angularVelocity = linearSpeed * Y

    return - angularVelocity


def isObstructed(pose, otherPose, lsr, lsrAngles):
    '''
    Uses the laser scanner to determine whether the path from the
    to otherPose is obstructed. Checks several angles, according to
    the width of the robot, ROBOT_WIDTH.Position
    Input: pose - Robot's current pose
           otherPose - The position to check
           lsr - A laser scan
           lsrAngles - The angles of the provided laser scan
    Output: True if there is an obstruction between the robot and the
            position, or if the position is outside scanning range.
            False if there is no obstruction.
    '''

    distanceToPosition = getDistance(
                pose['Pose']['Position'], otherPose['Pose']['Position'])

    angle = getAngle(pose, otherPose['Pose']['Position']);
    angleWidth = atan((ROBOT_WIDTH / 2) / distanceToPosition);

    startIndex = min(range(len(lsrAngles)),
            key=lambda i: abs(lsrAngles[i] - angle + angleWidth))
    endIndex = min(range(len(lsrAngles)),
            key=lambda i: abs(lsrAngles[i] - angle - angleWidth))


    # Position is outside the laser's scan angle
    if startIndex == 0 or endIndex == 270:
        return True

    for i in range(startIndex, endIndex):
        if lsr[i] < distanceToPosition :
            return True;

    return False

def setSpeedAndAvoidObstacles(pose, lsr, lsrAngles, angularSpeed, linearSpeed, carrotPosition):
    '''
    Adjusts the given angular and linear speeds in order to avoid obstacles
    found by using the laser scanner
    Input: pose - Robot's current pose
           lsr - A laser scan
           lsrAngles - The angles of the provided laser scan
           angularSpeed - The given angular speed
           linearSpeed - The given linear speed
    '''
    halfWidth = 29 # in indices
    zeroAngle = 135
    leftAngle = zeroAngle - halfWidth
    rightAngle = zeroAngle + halfWidth

    goalAngle = getAngle(pose, carrotPosition)
    goalAngleIdx = 0

    for idx, angle in enumerate(lsrAngles):
        if angle < goalAngle:
            goalAngleIdx = idx

    for i in range(leftAngle, rightAngle + 1):
        if lsr[i] < 1.3:
            if i < goalAngleIdx:
                angularSpeed += 2.0
            else:
                angularSpeed -= 2.0
            linearSpeed -= 0.7
            break;

    if linearSpeed < 0:
        linearSpeed = 0

    postSpeed(angularSpeed, linearSpeed)


def goFast(path, q_pure_exit=None):
    """
    Follow the path using pure pursuit along with algorithms to avoid
    obstacles and to take shortcuts if possible.
    Input: path - a path of nodes to follow
    """

    if q_pure_exit != None:
        q_pure_exit.get()

    lsrAngles = getLaserAngles()
    lsr = getLaser()['Echoes']
    pose = getPose()
    startNodeNum = 0
    curNodeNum = getNextCarrotNode(pose, startNodeNum, path, lsr, lsrAngles)
    nextPose = path[curNodeNum]['Pose']

    while not ((curNodeNum == len(path) - 1) and
            (getDistance(pose['Pose']['Position'], path[-1]['Pose']['Position']) < 1.0)):

        if q_pure_exit != None:
            if not q_pure_exit.empty():
                q_pure_exit.get()
                postSpeed(0, 0)
                logger.info("PURE PURSUIT RETURNED")
                return

        linearSpeed = 1.0 # max is 1
        
        angle = getAngle(pose, nextPose['Position'])
        if abs(angle)  > pi / 2:
            angularSpeed = -1.5 * (2 / pi) * angle
            linearSpeed = 0
        else:
            angularSpeed = getPureAngularSpeed(pose, nextPose['Position'], linearSpeed)
            #Slow down when doing sharp turns
            while abs(angularSpeed) > 1.8:
                linearSpeed -= 0.01
                angularSpeed = getPureAngularSpeed(pose, nextPose['Position'], linearSpeed)

        setSpeedAndAvoidObstacles(pose, lsr, lsrAngles, angularSpeed, linearSpeed, nextPose['Position'])

        time.sleep(0.04)

        pose = getPose()
        lsr = getLaser()['Echoes']

        curNodeNum = getNextCarrotNode(pose, curNodeNum, path, lsr, lsrAngles)
        newCarrotPosition = getNextCarrotPosition(pose, curNodeNum, path)
        nextPose = { 'Position' : newCarrotPosition }

        logger.info("Current node: " + str(curNodeNum) + " of " + str(len(path)))
        logger.info("distance to node: " + str(getDistance(pose['Pose']['Position'], path[curNodeNum]['Pose']['Position'])))

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
            "paths\\Path-to-and-from-bed.json",
            "paths\\Path-around-bench-and-sofa.json",
            "paths\\Path-around-table-and-sofa.json",
            "paths\\Path-around-table.json"
            ]

    for i in range(0, len(pathFiles) - 1):
    #Open path file
        with open(pathFiles[i]) as path_file:    
            path = json.load(path_file)

        start_time = time.time();
        goFast(path)
        end_time = time.time() - start_time
        print("Time for " + pathFiles[i] + ": " + str(end_time))
        with open(pathFiles[i + 1]) as path_file:    
            path = json.load(path_file)
        time.sleep(1)

    print("DONE")
    