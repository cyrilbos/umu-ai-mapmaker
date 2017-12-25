# TODO check if the position of the laser is equal to the robot's position

import logging
import numpy as np

from mapper.laser_model import LaserModel

logger = logging.getLogger(__name__)

class Mapper:
    """
    The cartographer module, responsible for updating the occupancy grid
    using the laser scanner.
    """

    def __init__(self, laser_angles, map):
        self._laser_angles = laser_angles  # these should be constant
        self._map = map
        self._laser_model = LaserModel(laser_angles)

    def update_map(self, laser_scan):
        """
        Update the occupancy values of the map using a sensor model
        and the provided laser scan.
        """
        pass



class Map:
    """
    An occupancy grid.
    """

    class OutsideMapException(Exception):
        pass

    #def __init__(self, real_width, real_height, scale):
    def __init__(self, x1, y1, x2, y2, scale):
        self._scale = scale  # "squares per meter"
        self._delta_cell = 1 / scale
        #self._real_width = real_width
        #self._real_height = real_height
        self._x1 = x1
        self._y1 = y1
        self._x2 = x2
        self._y2 = y2
        self._real_width = x2 - x1
        self._real_height = y2 - y1
        self._grid_width = int(self._real_width * scale)
        self._grid_height = int(self._real_height * scale)
        # self._grid = [[0.5 for x in range(self._grid_width)] for y in range(self._grid_height)]  # Bayesian init
        self._grid = np.empty((self._grid_width, self._grid_height))
        self._grid[:] = 0.5 # Bayesian init

    def grid(self):
        return self._grid

    def convert_to_grid_indexes(self, x, y):
        # return self.convert_to_grid_index(x), self.convert_to_grid_index(y)
        return self.convert_to_grid_index(x - self._x1), self.convert_to_grid_index(y - self._y1)

    def convert_to_grid_index(self, real_value):
        return int(real_value * self._scale)

    def convert_to_real_position(self, grid_x, grid_y):
        # return grid_x / self._scale, grid_y / self._scale
        return (grid_x / self._scale) + self._x2, (grid_y / self._scale) + self._y2

    def is_in_bounds(self, cell):
        return 0 <= cell[0] < self._grid_width and 0 <= cell[1] < self._grid_height

    def get_occupancy_idx(self, cell):
        if not self.is_in_bounds(cell):
            raise Map.OutsideMapException("trying to get the occupancy of a cell outside the grid")

        return self._grid[cell[0]][cell[1]]

    def set_occupancy_idx(self, cell, value):
        if not self.is_in_bounds(cell):
            raise Map.OutsideMapException("trying to get the occupancy of a cell outside the grid")

        self._grid[cell[0]][cell[1]] = value
        logger.debug("set grid[x:{}][y:{}]={}".format(cell[0], cell[1], value))

    def get_occupancy(self, x, y):
        if not self.is_in_bounds(self.convert_to_grid_indexes(x, y)):
            raise Map.OutsideMapException("trying to get the occupancy of a cell outside the grid")

        grid_x = int((x - self._x1) * self._scale)
        grid_y = int((y - self._y1) * self._scale)
        return self._grid[grid_x][grid_y]

    def set_occupancy(self, x, y, value):
        if not self.is_in_bounds(self.convert_to_grid_indexes(x, y)):
            raise Map.OutsideMapException("trying to set the occupancy of a cell outside the grid")
        
        grid_x = int((x - self._x1) * self._scale)
        grid_y = int((y - self._y1) * self._scale)
        self._grid[grid_x][grid_y] = value
        logger.debug("set grid[x:{}][y:{}]={}".format(grid_x, grid_y, value))

