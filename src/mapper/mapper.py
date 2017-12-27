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



