
# TODO check if the position of the laser is equal to the robot's position

import time
from util import *
from show_map import ShowMap

class Mapper:
    """
    The cartographer module, responsible for updating the occupancy grid
    using the laser scanner.
    """

    def __init__(self, laser_angles, map):
        self._laser_angles = laser_angles # these should be constant
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
        self._scale = scale # "squares per meter"
        self._real_width = real_width
        self._real_height = real_height
        map_width = int(real_width * scale)
        map_height = int(real_height * scale)
        print(map_width)
        print(map_height)
        self._map =  [[0 for x in range(map_width)] for y in range(map_height)]

    def get_occupancy(self, x, y):
        # TODO check bounds, raise exception
        grid_x = int(x / self._scale)
        grid_y = int(y / self._scale)
        return _map[grid_x][grid_y]

    def set_occupancy(self, x, y, value):
        # TODO check bounds, raise exception
        print(x)
        print(y)
        grid_x = int(x / self._scale)
        grid_y = int(y / self._scale)
        self._map[grid_x][grid_y] = value;


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

        ### TEMP ###
        robot_x = pose['Pose']['Position']['X']
        robot_y = pose['Pose']['Position']['Y']
        length = laser_scan['Echoes'][135] # straight forward
        angle = 0
        laser_hit_x = robot_x + length * cos(angle)
        laser_hit_y = robot_y + length * sin(angle)
        print("robot_x: " + str(robot_x) + " robot_y: " + str(robot_y))
        print("laser_x: " + str(laser_hit_x) + " laser_y: " + str(laser_hit_y))
        grid.set_occupancy(laser_hit_x, laser_hit_y, 15)
        ############

        # set occupancy around where the laser hit using some
        # probability algorithm?
        # set occupancy in a straight line to the laser hit, using
        # the line-drawing algorithm suggested in the specification
        pass

# Temporary testing code
if __name__ == '__main__':
    occupancy_map = Map(10, 10, 0.05)
    showmap_map = ShowMap(500, 500, True) # rows, cols, showgui
    laser_angles = getLaserAngles()
    laser_model = LaserModel(laser_angles)
    while True:
        laser_scan = getLaser()
        pose = getPose()
        laser_model.apply_model(occupancy_map, pose, laser_scan)
        showmap_map.updateMap(occupancy_map._map, 15, 0, 0)
        time.sleep(5)