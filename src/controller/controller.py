import http.client, json
from logging import getLogger
from math import atan2, cos, sin, pow, sqrt, pi
from threading import Lock
from time import sleep

from model import Vector, Quaternion
from model import pure_pursuit

logger = getLogger('controller')

# Headers sent with every POST speed requests
HEADERS = {"Content-type": "application/json", "Accept": "text/json"}


class Controller:
    """
    Controller base class which contains methods to send requests to the MRDS server and the travel monitoring
    routine.
    """

    class UnexpectedResponse(Exception):
        """
        Custom exception class raised when a HTTP request fails.
        """
        pass

    def __init__(self, mrds_url, lin_spd=0.5, delta_pos=0.75):
        """
        Initializes a new instance of Controller.
        :param mrds_url: url which the MRDS server listens on
        :type mrds_url: str
        :param lin_spd: linear speed value to automatically send in the requests
        :type lin_spd: float
        :param delta_pos: minimum distance to a point determine if the robot reached that point
        :type delta_pos: float

        """
        self.__mrds = http.client.HTTPConnection(mrds_url)
        self._lin_spd = lin_spd
        self._delta_pos = delta_pos

        logger.info('Controller set with linear speed={}, delta position={}'.format(lin_spd, delta_pos))

    def set_occupancy_map(self, occupancy_map):
        self._occupancy_map = occupancy_map

    def set_laser_model(self, laser_model):
        self._laser_model = laser_model

    def set_map_shower(self, map_shower):
        self._map_shower = map_shower


    def post_speed(self, angular_speed, linear_speed):
        """
        Sends a speed command to the MRDS server.

        :param angular_speed: value of angular speed
        :type angular_speed: float
        :param linear_speed: value of linear speed
        :type linear_speed: float
        """
        params = json.dumps({'TargetAngularSpeed': angular_speed, 'TargetLinearSpeed': linear_speed})
        self.__mrds.request('POST', '/lokarria/differentialdrive', params, HEADERS)
        response = self.__mrds.getresponse()
        status = response.status
        response.close()
        if status == 204:
            return response
        else:
            raise self.UnexpectedResponse(response)

    def get_pos(self):
        """
        Reads the current position from the MRDS and returns it as a Vector instance.
        """
        self.__mrds.request('GET', '/lokarria/localization')
        response = self.__mrds.getresponse()
        if response.status == 200:
            pos_data = json.loads(response.read().decode('utf-8'))
            response.close()
            return Vector.from_dict(pos_data['Pose']['Position'])
        else:
            raise self.UnexpectedResponse(response)

    def get_pos_and_orientation(self):
        """
        Reads the current position and orientation from the MRDS server and returns it as a tuple (Vector, Quaternion).
        """
        self.__mrds.request('GET', '/lokarria/localization')
        response = self.__mrds.getresponse()
        if response.status == 200:
            pos_data = json.loads(response.read().decode('utf-8'))
            response.close()
            return Vector.from_dict(pos_data['Pose']['Position']), Quaternion.from_dict(pos_data['Pose']['Orientation'])

        else:
            raise self.UnexpectedResponse(response)

    def get_laser_scan(self):
        """
        Requests the current laser scan from the MRDS server and parses it into a dict.
        """
        self.__mrds.request('GET', '/lokarria/laser/echoes')
        response = self.__mrds.getresponse()
        if response.status == 200:
            laser_data = response.read().decode('utf-8')
            response.close()
            return json.loads(laser_data)
        else:
            return self.UnexpectedResponse(response)

    def get_laser_scan_angles(self):
        """
        Requests the current laser properties from the MRDS server and returns a list of the laser angles.
        """
        self.__mrds.request('GET', '/lokarria/laser/properties')
        response = self.__mrds.getresponse()
        if response.status == 200:
            laser_data = response.read().decode('utf-8')
            response.close()
            properties = json.loads(laser_data)
            beamCount = int((properties['EndAngle'] - properties['StartAngle']) / properties['AngleIncrement'])
            a = properties['StartAngle']  # +properties['AngleIncrement']
            angles = []
            while a <= properties['EndAngle']:
                angles.append(a)
                a += pi / 180  # properties['AngleIncrement']
            # angles.append(properties['EndAngle']-properties['AngleIncrement']/2)
            return angles
        else:
            raise self.UnexpectedResponse(response)

    def set_pos_path(self, pos_path):
        logger.info("Set new path to goal {}".format(pos_path[len(pos_path)-1]))
        self._pos_path = pos_path

    def get_pos_path(self):
        return self._pos_path