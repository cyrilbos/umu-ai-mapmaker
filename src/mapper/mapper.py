# TODO check if the position of the laser is equal to the robot's position

import logging

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

    def __init__(self, real_width, real_height, scale):
        self._scale = scale  # "squares per meter"
        self._real_width = real_width
        self._real_height = real_height
        map_width = int(real_width * scale)
        map_height = int(real_height * scale)
        self._grid = [[0 for x in range(map_width)] for y in range(map_height)]

    def grid(self):
        return self._grid

    def convert_to_grid_indexes(self, x, y):
        return self.convert_to_grid_index(x), self.convert_to_grid_index(y)

    def convert_to_grid_index(self, real_value):
        return round(real_value * self._scale)

    def convert_to_real_position(self, grid_x, grid_y):
        return grid_x / self._scale, grid_y / self._scale

    def get_occupancy(self, x, y):
        # TODO check bounds, raise exception
        grid_x = round(x * self._scale)
        grid_y = round(y * self._scale)
        return self._grid[grid_x][grid_y]

    def set_occupancy(self, x, y, value):
        # TODO check bounds, raise exception
        if x < 0 or y < 0 or x > self._real_width or y > self._real_height:
            return
        
        grid_x = round(x * self._scale)
        grid_y = round(y * self._scale)
        self._grid[grid_x][grid_y] = value
        logger.debug("set grid[x:{}][y:{}]={}".format(grid_x, grid_y, value))

