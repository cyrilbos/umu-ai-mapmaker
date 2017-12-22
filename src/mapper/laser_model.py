from logging import getLogger
from math import atan2, pi, cos, sin

import math

import numpy as np

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

    def bayesian_probability(self, occupied_probability, previous_probabilty):
        empty_probability = 1 - occupied_probability
        empty_previous_probability = 1 - previous_probabilty
        return ((occupied_probability * previous_probabilty) /
                (occupied_probability * previous_probabilty + empty_probability * empty_previous_probability))


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
            # region 1 = hit cell
            hit_cell = grid.convert_to_grid_indexes(laser_hit_x, laser_hit_y)
            # alpha angle = 0
            # occupied_probability = (((R - r) / R) + 1) / 2 * self._p_max
            # grid.set_occupancy(laser_hit_x, laser_hit_y, occupied_probability)

            # logger.info("probability hit cell {}".format(occupied_probability))

            logger.info("hit cell [{}][{}]".format(hit_cell[0], hit_cell[1]))
            ############ seems that beta = 0.5 degrees is not enough of a difference to get a different cell
            # TODO: might want to try with a higher resolution map
            """ 
            below_hit_x = robot_pos.x + distance * cos(angle + beta)
            below_hit_y = robot_pos.y + distance * sin(angle + beta)
            below_hit_cell = grid.convert_to_grid_indexes(below_hit_x, below_hit_y)
            logger.info("below hit cell [{}][{}]".format(below_hit_cell[0], below_hit_cell[1]))
            above_hit_x = robot_pos.x + distance * cos(angle - beta)
            above_hit_y = robot_pos.y + distance * sin(angle - beta)
            above_hit_cell = grid.convert_to_grid_indexes(above_hit_x, above_hit_y)
            logger.info("above hit cell [{}][{}]".format(above_hit_cell[0], above_hit_cell[1]))
    
            # TODO: recursive dichotomy? while above / below hit cell != hit cell
            # TODO: go from alpha + beta to alpha = beta, decrement by distance between 2?
            occupied_probability = (((R - r) / R) + ((beta - beta) / beta)) / 2 * self._p_max
            previous_probabilty = grid.get_occupancy(float(x), float(y))
            
            self.set_occupancy(laser_hit_x, laser_hit_y, self.bayesian_probability(occupied_probability, previous_probabilty))
            
            if below_hit_cell != hit_cell:
                grid.set_occupancy(below_hit_x, below_hit_y, occupied_probability)
            if above_hit_cell != hit_cell:
                grid.set_occupancy(above_hit_x, above_hit_y,
                                   occupied_probability)  # same occupied probability for both?
            """

            # region 2 = cells between the robot cell and the hit cell
            self.bresenham_line(hit_cell, robot_cell, grid, R)
                # set occupancy in a straight line to the laser hit, using
                # the line-drawing algorithm suggested in the specification

    def bresenham_line(self, hit_cell, robot_cell, grid, R):
        deltax = hit_cell[0] - robot_cell[0]
        deltay = hit_cell[1] - robot_cell[1]
        deltaerr = abs(deltay / deltax) if deltax != 0 else 0  # Assume deltax != 0 (line is not vertical),
        # note that this division needs to be done in a way that preserves the fractional part
        error = 0.0  # No error at start
        y = robot_cell[1]
        updated_cells = []

        for x in range(robot_cell[0], hit_cell[0], int(math.copysign(1, deltay))):
            cell = (int(x),int(y))
            if cell not in updated_cells and grid.is_in_bounds(hit_cell):
                # fx = float(x)  # converting from numpy float to use Python round() method
                # fy = float(y)
                # TODO: check if that is true, otherwise angle is computable
                # alpha angle is supposed to be 0 (straight line), so beta - 0 / beta = 1
                r = np.linalg.norm(cell[0] - robot_cell[0]) #euclidian distance between cell and robot
                occupied_probability = (((R - r) / R) + 1) / 2 * self._p_max
                previous_probability = grid.get_occupancy_idx(cell)
                # empty probability, so passing 1 - occupied_probability
                grid.set_occupancy_idx(cell, self.bayesian_probability(1 - occupied_probability,
                                                                         previous_probability))
                updated_cells.append(cell)
            error = error + deltaerr
            while error >= 0.5:
                y = y + math.copysign(1, deltay)  # math.copysign(1,x) means sign(x)
                error -= 1.0
