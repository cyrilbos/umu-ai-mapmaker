from logging import getLogger
from math import atan2, pi, cos, sin, hypot

import math

from controller.util import heading

logger = getLogger(__name__)


# Got these values from requesting the laser properties (/lokarria/laser/properties).
# I incorrectly assumed that the position of the laser scanner were the same as the
# position of the robot. This should make scans more accurate.
# The Z value is 0.2, in case it is needed for some reason.
LASER_OFFSET_X = 0.15
LASER_OFFSET_Y = 0


class LaserModel:
    """
    Implements a sensor model by providing a function
    that updates a given map using laser scanner data.
    """

    def __init__(self, laser_angles, laser_max_distance):
        self._laser_angles = laser_angles
        self._max_distance = laser_max_distance
        self._p_max = 0.98
        self._minimum_increase = 0.01

    @property
    def p_max(self):
        return self._p_max

    def bayesian_probability(self, occupied_probability, previous_probabilty):
        empty_probability = 1 - occupied_probability
        empty_previous_probability = 1 - previous_probabilty
        return ((occupied_probability * previous_probabilty) /
                (occupied_probability * previous_probabilty + empty_probability * empty_previous_probability))


    def apply_model(self, grid, robot_pos, robot_orientation, laser_scan):
        # use the laser scan to update the map using
        # the sensor model

        cur_heading = heading(robot_orientation)
        robot_angle = atan2(cur_heading['Y'], cur_heading['X'])

        # fixes weird angle bug
        while robot_angle > pi:
            robot_angle -= pi
        while robot_angle < -pi:
            robot_angle += pi

        distance_to_laser = LASER_OFFSET_X # Since the y-offset is zero

        laser_pos_x = robot_pos['X'] + distance_to_laser * cos(robot_angle)
        laser_pos_y = robot_pos['Y'] + distance_to_laser * sin(robot_angle)

        logger.debug("robot pos x:{} y:{}".format(robot_pos['X'], robot_pos['Y']))

        R = self._max_distance

        for idx, laser_angle in enumerate(self._laser_angles):
            if idx % 2 == 0:
                continue
            r = distance = laser_scan['Echoes'][idx]

            angle = robot_angle + laser_angle

            logger.debug("laser index: {}".format(idx))
            
            laser_hit_x = laser_pos_x + distance * cos(angle)
            laser_hit_y = laser_pos_y + distance * sin(angle)
            logger.debug("laser angle: {}".format(laser_angle))

            logger.debug("robot_pos.x: " + str(robot_pos['X']) + " robot_pos.y: " + str(robot_pos['Y']))
            logger.debug("laser_x: " + str(laser_hit_x) + " laser_y: " + str(laser_hit_y))

            hit_cell = grid.convert_to_grid_indexes(laser_hit_x, laser_hit_y)
            logger.debug("hit cell [{}][{}]".format(hit_cell[0], hit_cell[1]))

            robot_cell = grid.convert_to_grid_indexes(robot_pos['X'], robot_pos['Y'])
      
            # region 2 = cells between the robot cell and the hit cell
            self.bresenham_line(hit_cell, robot_cell, grid, R)

            if grid.is_in_bounds(hit_cell) and distance < R - 10:
                logger.debug("hit cell [{}][{}]".format(robot_cell[0], robot_cell[1]))
                # region 1 = hit cell
                # alpha angle = 0
                occupied_probability = ((((R - r) / R) + 1) / 2 * self._p_max) + self._minimum_increase
                grid.set_occupancy(laser_hit_x, laser_hit_y, self.bayesian_probability(occupied_probability,
                                                                                       grid.get_occupancy(laser_hit_x, laser_hit_y)))

    def bresenham_line(self, hit_cell, robot_cell, grid, R):
        deltax = hit_cell[0] - robot_cell[0]
        deltay = hit_cell[1] - robot_cell[1]
        deltaerr = abs(deltay / deltax) if deltax != 0 else 0  # Assume deltax != 0 (line is not vertical),
        # note that this division needs to be done in a way that preserves the fractional part
        error = 0.0  # No error at start
        y = robot_cell[1]
        updated_cells = []

        for x in range(robot_cell[0], hit_cell[0], int(math.copysign(1, deltax))):
            cell = (int(x),int(y))
            if x == hit_cell[0] + int(math.copysign(1, deltax)) * 5:
                return
            if cell not in updated_cells and grid.is_in_bounds(cell):

                # alpha angle is supposed to be 0 (straight line), so beta - 0 / beta = 1
                real_cell = grid.convert_to_real_position(cell[0], cell[1])
                real_robot_cell = grid.convert_to_real_position(robot_cell[0], robot_cell[1])
                r = hypot(real_cell[0] - real_robot_cell[0], real_cell[1] - real_robot_cell[1])

                if r < R - 10:
                    occupied_probability = (((R - r) / R) + 1) / 2 * self._p_max
                    previous_probability = grid.get_occupancy_idx(cell)
                    # empty probability, so passing 1 - occupied_probability
                    grid.set_occupancy_idx(cell, self.bayesian_probability(1 - occupied_probability, previous_probability))
                    updated_cells.append(cell)
            error = error + deltaerr
            while error >= 0.5:
                y = y + math.copysign(1, deltay)  # math.copysign(1,x) means sign(x)
                error -= 1.0
