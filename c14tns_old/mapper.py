
# TODO check if the position of the laser is equal to the robot's position


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
        map_width = real_width * scale
        map_height = real_height * scale
        self._map =  [[0 for x in range(map_width)] for y in range(map_height)]

    def get_occupancy(x, y):
        # TODO check bounds, raise exception
        grid_x = x / scale
        grid_y = y / scale
        return _map[grid_x][grid_y]

    def set_occupancy(x, y, value):
        # TODO check bounds, raise exception
        grid_x = x / scale
        grid_y = y / scale
        _map[grid_x][grid_y] = value;


class LaserModel:
    """
    Implements a sensor model by providing a function
    that updates a given map using laser scanner data.
    """

    def __init__(self, laser_angles):
        self._laser_angles = laser_angles

    def apply_model(map, pose, laser_scan):
        # use the laser scan to update the map using
        # the sensor model


        ### TEMP ###
        robot_x = ...
        robot_y = ...
        length = laser_scan[135]
        angle = 0
        laser_hit_x = robot_x + length * sin(angle)
        laser_hit_y = robot_y + length * cos(angle)
        ############

        # set occupancy around where the laser hit using some
        # probability algorithm?
        # set occupancy in a straight line to the laser hit, using
        # the line-drawing algorithm suggested in the specification
        pass
