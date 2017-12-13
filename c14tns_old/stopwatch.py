"""
Stopwatch for Microsoft Robotic Developer Studio 4 via the Lokarria http interface.

This program will track the position of the robot and measure the time it takes to complete a given path.

Author: Erik Billing (billing@cs.umu.se)
"""

START_THRESHOLD = 0.1
PASS_THRESHOLD = 1.5
GOAL_THRESHOLD = 1
MRDS_URL = 'localhost:50000'

import math, time, json, sys
import httplib, json, time
from math import sin,cos,pi,atan2

HEADERS = {"Content-type": "application/json", "Accept": "text/json"}

class UnexpectedResponse(Exception): pass

def postSpeed(angularSpeed,linearSpeed):
    """Sends a speed command to the MRDS server"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    params = json.dumps({'TargetAngularSpeed':angularSpeed,'TargetLinearSpeed':linearSpeed})
    mrds.request('POST','/lokarria/differentialdrive',params,HEADERS)
    response = mrds.getresponse()
    status = response.status
    #response.close()
    if status == 204:
        return response
    else:
        raise UnexpectedResponse(response)

def getLaser():
    """Requests the current laser scan from the MRDS server and parses it into a dict"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/laser/echoes')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        return json.loads(laserData)
    else:
        return response

def getLaserAngles():
    """Requests the current laser properties from the MRDS server and parses it into a dict"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/laser/properties')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        properties = json.loads(laserData)
        beamCount = int((properties['EndAngle']-properties['StartAngle'])/properties['AngleIncrement'])+1
        a = properties['StartAngle']+properties['AngleIncrement']/2
        angles = []
        while a < properties['EndAngle']:
            angles.append(a)
            a+=properties['AngleIncrement']
        angles.append(properties['EndAngle']-properties['AngleIncrement']/2)
        return angles
    else:
        raise UnexpectedResponse(response)

def getPose():
    """Reads the current position and orientation from the MRDS"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/localization')
    response = mrds.getresponse()
    if (response.status == 200):
        poseData = response.read()
        response.close()
        return json.loads(poseData)
    else:
        return UnexpectedResponse(response)

def bearing(q):
    return rotate(q,{'X':1.0,'Y':0.0,"Z":0.0})

def rotate(q,v):
    return vector(qmult(qmult(q,quaternion(v)),conjugate(q)))

def quaternion(v):
    q=v.copy()
    q['W']=0.0;
    return q

def vector(q):
    v={}
    v["X"]=q["X"]
    v["Y"]=q["Y"]
    v["Z"]=q["Z"]
    return v

def conjugate(q):
    qc=q.copy()
    qc["X"]=-q["X"]
    qc["Y"]=-q["Y"]
    qc["Z"]=-q["Z"]
    return qc

def qmult(q1,q2):
    q={}
    q["W"]=q1["W"]*q2["W"]-q1["X"]*q2["X"]-q1["Y"]*q2["Y"]-q1["Z"]*q2["Z"]
    q["X"]=q1["W"]*q2["X"]+q1["X"]*q2["W"]+q1["Y"]*q2["Z"]-q1["Z"]*q2["Y"]
    q["Y"]=q1["W"]*q2["Y"]-q1["X"]*q2["Z"]+q1["Y"]*q2["W"]+q1["Z"]*q2["X"]
    q["Z"]=q1["W"]*q2["Z"]+q1["X"]*q2["Y"]-q1["Y"]*q2["X"]+q1["Z"]*q2["W"]
    return q

def getBearing():
    """Returns the XY Orientation as a bearing unit vector"""
    return bearing(getPose()['Pose']['Orientation'])

class Stopwatch:
    def __init__(self,path):
        self.path = path
        self.startPose = getPose()
        self.startTime = None

    def run(self):
        print 'Waiting for robot to start the race...'
        while xyDistance(self.startPose,getPose()) < START_THRESHOLD:
            time.sleep(0.01)
        self.startTime = time.time()
        print 'GO!'
        totCount = float(len(self.path))
        for i,point in enumerate(self.path):
            self.passPoint(point)
            print '%.0f%% of the path completed: %.2f seconds'%(i/totCount*100,time.time()-self.startTime)
        print 'All points passed, looking for goal point...'
        self.passPoint(self.path[-1],GOAL_THRESHOLD)
        self.goalTime = time.time()
        print 'Goal reached in %.2f seconds.'%(self.goalTime-self.startTime)

    def passPoint(self,pose,threshold=PASS_THRESHOLD):
        while xyDistance(pose,getPose()) > threshold:
            time.sleep(0.01)

def xyDistance(pose1,pose2):
    pos1, pos2 = pose1['Pose']['Position'], pose2['Pose']['Position']
    x1, y1 = pos1['X'], pos1['Y']
    x2, y2 = pos2['X'], pos2['Y']
    dx = x1-x2
    dy = y1-y2
    return math.sqrt(dx*dx+dy*dy)

if __name__ == '__main__':
    stopwatch = Stopwatch(json.load(open(sys.argv[1])))
    stopwatch.run()