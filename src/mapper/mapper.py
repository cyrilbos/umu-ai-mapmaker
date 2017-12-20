# TODO check if the position of the laser is equal to the robot's position

import logging

from mapper.LaserModel import LaserModel

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

    def update_map(laser_scan):
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
        print(map_width)
        print(map_height)
        self._map = [[0 for x in range(map_width)] for y in range(map_height)]

    def get_occupancy(self, x, y):
        # TODO check bounds, raise exception
        grid_x = round(x * self._scale)
        grid_y = round(y * self._scale)
        return self._map[grid_x][grid_y]

    def set_occupancy(self, x, y, value):
        # TODO check bounds, raise exception
        if x < 0 or y < 0 or x > 40 or y > 40:
            return
        
        grid_x = round(x * self._scale)
        grid_y = round(y * self._scale)
        print(grid_x)
        print(grid_y)
        self._map[grid_x][grid_y] = value

