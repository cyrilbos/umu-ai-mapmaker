#!/usr/bin/env python3
"""
A robot controller for MRDS.

Follows a given path using an implementation of the pure pursuit
algorithm.

Authors: Ville Gillström (oi14vgm@cs.umu.se)
         Tobias Nebaeus (c14tns@cs.umu.se)

"""
import winsound

from logging import getLogger
from math import atan

from controller.util import *

ROBOT_WIDTH = 0.9 # actual is 0.4

logger = getLogger(__name__)

g_lookahead = 2.0
g_blocked_times = 0

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

    return hypot(dy, dx)
    

def get_angle(pose, nextPosition):
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


def getNextCarrotNode(pose, cur_node_num, path):
    '''
    Get the next node to "aim" at
    Input:  pose - Robot's current pose
            curNodeNum - Index of the current goal node
            path - The path to follow
    Output: The carrot (goal) node
    '''

    while (isWithinLOOKAHEAD(pose['Pose']['Position'],
                             path[cur_node_num]['Pose']['Position']) and
           (cur_node_num < len(path) - 1)):
        cur_node_num += 1
    
    return cur_node_num


def getNextCarrotPosition(pose, cur_node_num, path):
    '''
    Get the exact position of the carrot point on the path segment
    between node curNodeNum and the next one
    Input:  pose - Robot's current pose
            curNodeNum - Index of the current goal node
            path - The path to follow
    Output: The carrot (goal) position
    '''

    if cur_node_num == len(path) - 1:
        return path[cur_node_num]['Pose']['Position']

    x1 = path[cur_node_num]['Pose']['Position']['X']
    x2 = path[cur_node_num + 1]['Pose']['Position']['X']
    y1 = path[cur_node_num]['Pose']['Position']['Y']
    y2 = path[cur_node_num + 1]['Pose']['Position']['Y']

    distanceToAdd = g_lookahead - getDistance(pose['Pose']['Position'],
                                              path[cur_node_num]['Pose']['Position'])

    dy = y2 - y1
    dx = x2 - x1
    if dx == 0:
        xToAdd = 0
        yToAdd = distanceToAdd
    else:
        dydx = dy / dx
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
    y = sin(get_angle(pose, goalPosition)) * L
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

    angle = get_angle(pose, otherPose['Pose']['Position'])
    angleWidth = atan((ROBOT_WIDTH / 2) / distanceToPosition)

    startIndex = min(range(len(lsrAngles)),
            key=lambda i: abs(lsrAngles[i] - angle + angleWidth))
    endIndex = min(range(len(lsrAngles)),
            key=lambda i: abs(lsrAngles[i] - angle - angleWidth))


    # Position is outside the laser's scan angle
    if startIndex == 0 or endIndex == 270:
        return True

    for i in range(startIndex, endIndex):
        if lsr[i] < distanceToPosition:
            return True

    return False


def stop(mrds_url):

    postSpeed(mrds_url, 0, 0)
    time.sleep(1)

def set_speed_and_avoid_obstacles(mrds_url, sound, angular_speed, linear_speed):
    """
    Adjusts the given angular and linear speeds in order to avoid obstacles
    found by using the laser scanner
    Input: pose - Robot's current pose
           lsr - A laser scan
           lsrAngles - The angles of the provided laser scan
           angularSpeed - The given angular speed
           linearSpeed - The given linear speed
    """
    global g_blocked_times
    blocked_distance = (1 + g_blocked_times) * ROBOT_WIDTH / 1.5
    blocked, is_right_angle = is_blocked(mrds_url, blocked_distance)
    if blocked:
        angular_speed, linear_speed = blocked_speeds(is_right_angle)
        stop(mrds_url)
        if unblock(mrds_url, is_right_angle, blocked_distance):
            return True #was going to hit a wall by going backwards, so replan
        g_blocked_times += 1

        if g_blocked_times > 3:
            g_blocked_times = 0
            stop(mrds_url)
            if sound:
                winsound.Beep(2000, 50)
            return True #blocked too many times, so replan

    postSpeed(mrds_url, angular_speed, linear_speed)


def is_blocked(mrds_url, blocked_distance):
    half_width = 20  # in indices
    zero_angle = 135

    left_angle = zero_angle - half_width
    right_angle = zero_angle + half_width
    lsr = getLaser(mrds_url)['Echoes']
    blocked = False

    shortest_distance = 10
    is_right_angle = False

    for i in range(left_angle, zero_angle):
        if lsr[i] < blocked_distance:
            blocked, is_right_angle, shortest_distance = handle_blocking_laser(blocked, False, lsr[i],
                                                                               shortest_distance)

    for i in range(zero_angle, right_angle):
        if lsr[i] < blocked_distance:
            blocked, is_right_angle, shortest_distance = handle_blocking_laser(blocked, True, lsr[i],
                                                                               shortest_distance)

    return blocked, is_right_angle

def blocked_speeds(is_right_angle):
    if is_right_angle:
        angular_speed = -1
    else:
        angular_speed = 1
    linear_speed = -0.6
    return 0, linear_speed

def unblock(mrds_url, is_right_angle, blocked_distance):
    global g_blocked_times
    start_pos = getPose(mrds_url)['Pose']['Position']
    blocked = True
    angular_speed, linear_speed = blocked_speeds(is_right_angle)
    postSpeed(mrds_url, angular_speed, linear_speed)
    time.sleep(0.5)
    while blocked:
        current_pos = getPose(mrds_url)['Pose']['Position']

        blocked, new_is_right_angle = is_blocked(mrds_url, blocked_distance)
        if blocked and new_is_right_angle != is_right_angle:
            angular_speed, linear_speed = blocked_speeds(is_right_angle)

        start_pos = current_pos
        postSpeed(mrds_url, angular_speed, linear_speed)
        time.sleep(0.5)

    return False

def handle_blocking_laser(blocked, is_right_angle, lsr_dist, shortest_distance):
    if not blocked:
        blocked = True
    if lsr_dist < shortest_distance:
        shortest_distance = lsr_dist
    return blocked, is_right_angle, shortest_distance


def goFast(path, mrds_url, sound=False):
    """
    Follow the path using pure pursuit along with algorithms to avoid
    obstacles and to take shortcuts if possible.
    Input: path - a path of nodes to follow
    """

    lsrAngles = getLaserAngles(mrds_url)
    lsr = getLaser(mrds_url)['Echoes']
    pose = getPose(mrds_url)
    startNodeNum = 0
    if len(path) > 1:
        cur_node_num = getNextCarrotNode(pose, startNodeNum, path)
    else:
        cur_node_num = startNodeNum

    next_pose = path[cur_node_num]['Pose']
    while not ((cur_node_num == len(path) - 1) and
            (getDistance(pose['Pose']['Position'], path[-1]['Pose']['Position']) < ROBOT_WIDTH)):

        linear_speed = 1.0 # max is 1
        
        angle = get_angle(pose, next_pose['Position'])
        #TODO: put old if and put this special case outside the while on first path pos
        while abs(angle) > pi / 8:
            angular_speed = -3 * angle
            postSpeed(mrds_url, angular_speed, 0.3)
            time.sleep(0.1)
            angle = get_angle(getPose(mrds_url), next_pose['Position'])

        angular_speed = getPureAngularSpeed(pose, next_pose['Position'], linear_speed)
        #Slow down when doing sharp turns
        while abs(angular_speed) > 1.8:
            linear_speed -= 0.01
            angular_speed = getPureAngularSpeed(pose, next_pose['Position'], linear_speed)

        if set_speed_and_avoid_obstacles(mrds_url, sound, angular_speed, linear_speed):
            return False  # blocked, so stop and replan

        time.sleep(0.04)

        pose = getPose(mrds_url)
        lsr = getLaser(mrds_url)['Echoes']

        cur_node_num = getNextCarrotNode(pose, cur_node_num, path)
        logger.debug("CurNodeNum: " + str(cur_node_num) + " of " + str(len(path)))
        new_carrot_position = getNextCarrotPosition(pose, cur_node_num, path)
        next_pose = { 'Position' : new_carrot_position }

        logger.debug("Current node: " + str(cur_node_num) + " of " + str(len(path)))
        logger.debug("distance to node: " + str(getDistance(pose['Pose']['Position'], path[cur_node_num]['Pose']['Position'])))

    postSpeed(mrds_url, 0, 0)