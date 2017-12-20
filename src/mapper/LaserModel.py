from logging import getLogger
from math import atan2, pi, cos, sin

from .util import heading

logger = getLogger(__name__)

class LaserModel:
    """
    Implements a sensor model by providing a function
    that updates a given map using laser scanner data.
    """

    def __init__(self, laser_angles):
        self._laser_angles = laser_angles

    def apply_model(self, grid, pose, laser_scan):
        # use the laser scan to update the map using
        # the sensor model

        robot_x = pose['Pose']['Position']['X']
        robot_y = pose['Pose']['Position']['Y']
        cur_heading = heading(pose['Pose']['Orientation'])
        robot_angle = atan2(cur_heading['Y'], cur_heading['X'])

        beta = 0.5  # degrees, to be optimized
        r = grid._map

        #TODO: sensor model

        for idx, laser_angle in enumerate(self._laser_angles):
        #idx = 200
        #laser_angle = self._laser_angles[200]
            R = length = laser_scan['Echoes'][idx]

            angle = robot_angle + laser_angle
            logger.debug("laser index: {}".format(idx))
            logger.debug("laser angle: {}".format(angle))

            # fixes weird angle bug
            while angle > pi:
                angle -= pi
            while angle < -pi:
                angle += pi

            laser_hit_x = robot_x + length * cos(angle)
            laser_hit_y = robot_y + length * sin(angle)
            logger.debug(laser_hit_x)
            logger.debug(laser_hit_y)
            logger.debug("robot_x: " + str(robot_x) + " robot_y: " + str(robot_y))
            logger.debug("laser_x: " + str(laser_hit_x) + " laser_y: " + str(laser_hit_y))

            #TODO:
            occupied_probability = 15
            grid.set_occupancy(laser_hit_x, laser_hit_y, occupied_probability)



        ############

        # set occupancy around where the laser hit using some
        # probability algorithm?
        # set occupancy in a straight line to the laser hit, using
        # the line-drawing algorithm suggested in the specification
        pass