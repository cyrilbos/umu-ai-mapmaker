from logging import getLogger
from math import atan2, pi, cos, sin

from model import Quaternion, Vector
from .util import heading

logger = getLogger(__name__)


class LaserModel:
    """
    Implements a sensor model by providing a function
    that updates a given map using laser scanner data.
    """

    def __init__(self, laser_angles, max_distance):
        self._laser_angles = laser_angles
        self._max_distance = max_distance
        self._p_max = 1

    def apply_model(self, grid, robot_pos, robot_orientation, laser_scan):
        # use the laser scan to update the map using
        # the sensor model

        cur_heading = Quaternion(robot_orientation.w, Vector(0, 0, robot_orientation.z)).heading()
        robot_angle = atan2(cur_heading.y, cur_heading.x)

        logger.debug("robot pos x:{} y:{}".format(robot_pos.x, robot_pos.y))

        # TODO: use beta in controller laser scan
        beta = 0.5 * pi / 180  # degrees, to be optimized
        print(beta)

        R = self._max_distance

        for idx, laser_angle in enumerate(self._laser_angles):
            # idx = 200
            # laser_angle = self._laser_angles[200]
            r = distance = laser_scan['Echoes'][idx]
            if distance > self._max_distance:
                continue

            angle = robot_angle + laser_angle
            logger.debug("laser index: {}".format(idx))
            logger.debug("laser angle: {}".format(angle))

            # fixes weird angle bug
            while angle > pi:
                angle -= pi
            while angle < -pi:
                angle += pi

            laser_hit_x = robot_pos.x + distance * cos(angle)
            laser_hit_y = robot_pos.y + distance * sin(angle)
            logger.debug("robot_pos.x: " + str(robot_pos.x) + " robot_pos.y: " + str(robot_pos.y))
            logger.debug("laser_x: " + str(laser_hit_x) + " laser_y: " + str(laser_hit_y))

            robot_cell = grid.convert_to_grid_indexes(robot_pos.x, robot_pos.y)
            logger.info(robot_cell)
            # region 1
            # alpha = 0
            hit_cell = grid.convert_to_grid_indexes(laser_hit_x, laser_hit_y)
            occupied_probability = ((R - r) / R + 1) / 2 * self._p_max
            grid.set_occupancy(laser_hit_x, laser_hit_y, occupied_probability)

            logger.info("probability hit cell {}".format(occupied_probability))
            """
            logger.info("hit cell [{}][{}]".format(hit_cell[0], hit_cell[1]))
            below_hit_x = robot_pos.x + distance * cos(angle + beta)
            below_hit_y = robot_pos.y + distance * sin(angle + beta)
            below_hit_cell = grid.convert_to_grid_indexes(below_hit_x, below_hit_y)
            logger.info("below hit cell [{}][{}]".format(below_hit_cell[0], below_hit_cell[1]))
            above_hit_x = robot_pos.x + distance * cos(angle - beta)
            above_hit_y = robot_pos.y + distance * sin(angle - beta)
            above_hit_cell = grid.convert_to_grid_indexes(above_hit_x, above_hit_y)
            logger.info("above hit cell [{}][{}]".format(above_hit_cell[0], above_hit_cell[1]))

            # TODO: recursive dichotomy? until above / below hit cell == hit cell
            # TODO: go from alpha + beta to alpha = beta, decrement by distance between 2?
            occupied_probability = (((R - r) / R) + ((beta - beta) / beta)) / 2 * self._p_max

            if below_hit_cell != hit_cell:
                grid.set_occupancy(below_hit_x, below_hit_y, occupied_probability)
            if above_hit_cell != hit_cell:
                grid.set_occupancy(above_hit_x, above_hit_y,
                                   occupied_probability)  # same occupied probability for both?

            # region 2
            # do the same for cell between robot and laser distance
            #grid.set_occupancy(laser_hit_x, laser_hit_y, 1 - occupied_probability)
            """
        ############
        # set occupancy in a straight line to the laser hit, using
        # the line-drawing algorithm suggested in the specification
