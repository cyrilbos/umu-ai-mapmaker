import numpy as np

from mapper.mapper import logger


class Map:
    """
    An occupancy grid.
    """

    class OutsideMapException(Exception):
        pass

    #def __init__(self, real_width, real_height, scale):
    def __init__(self, x1, y1, x2, y2, scale, init_robot_pos):
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

        # starting world position of the robot, to translate grid position into world position to travel to
        self._init_robot_pos = init_robot_pos

    @property
    def grid_width(self):
        return self._grid_width

    @property
    def grid_height(self):
        return self._grid_height

    @property
    def grid(self):
        return self._grid

    def convert_to_grid_indexes(self, x, y):
        # return self.convert_to_grid_index(x), self.convert_to_grid_index(y)
        return self.convert_to_grid_index(x - self._x1), self.convert_to_grid_index(y - self._y1)

    def convert_to_grid_index(self, real_value):
        return int(real_value * self._scale)

    def convert_to_real_position(self, grid_x, grid_y):
        # return grid_x / self._scale, grid_y / self._scale
        return (grid_x / self._scale) + self._x1, (grid_y / self._scale) + self._y1

    def convert_to_world_position(self, cell):
        cell_pos = self.convert_to_real_position(*cell)
        return cell_pos[0] + self._init_robot_pos.x, cell_pos[1] + self._init_robot_pos.y

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

    def is_an_obstacle(self, cell):
        return self._grid[cell[0]][cell[1]] > 0.7