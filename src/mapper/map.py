import numpy as np

from logging import getLogger

logger = getLogger(__name__)


class Map:
    """
    An occupancy grid.
    """

    class OutsideMapException(Exception):
        pass

    def __init__(self, x1, y1, x2, y2, scale):
        """

        :param x1: x coordinate of top left corner of the area to explore
        :param y1: y coordinate of top left corner of the area to explore
        :param x2: x coordinate of bottom right corner of the area to explore
        :param y2: y coordinate of top left corner of the area to explore
        :param scale: resolution, i.e the number of cspace cells per real world meter
        """
        self._scale = scale  # "squares per meter"
        self._delta_cell = 1 / scale

        self._x1 = x1
        self._y1 = y1
        self._x2 = x2
        self._y2 = y2
        self._real_width = x2 - x1
        self._real_height = y2 - y1
        self._grid_width = int(self._real_width * scale)
        self._grid_height = int(self._real_height * scale)
        self._grid = np.empty((self._grid_width, self._grid_height))
        self._grid[:] = 0.5  # Bayesian init

    @property
    def grid_width(self):
        return self._grid_width

    @property
    def grid_height(self):
        return self._grid_height

    @property
    def grid(self):
        return self._grid

    def center_of_cell(self, x, y):
        """
        returns the world position of the center of a cell of the cspace grid, to avoid using
        the corner.
        """
        xr, yr = self.convert_to_real_position(x, y)
        return (xr + (1 / (self._scale) * 2), yr + (1 / (self._scale) * 2))

    def convert_to_grid_indexes(self, x, y):
        # return self.convert_to_grid_index(x), self.convert_to_grid_index(y)
        return self.convert_to_grid_index(x - self._x1), self.convert_to_grid_index(y - self._y1)

    def convert_to_grid_index(self, real_value):
        return int(real_value * self._scale)

    def convert_to_real_position(self, grid_x, grid_y):
        #return grid_x / self._scale, grid_y / self._scale
        return (grid_x / self._scale) + self._x1, (grid_y / self._scale) + self._y1

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

    def is_unexplored(self, cell):
        return self._grid[cell[0]][cell[1]] == 0.5

    def is_an_obstacle(self, cell):
        return self._grid[cell[0]][cell[1]] > 0.5

    def obstacle_expanded_map(self):
        """
            Returns a new instance of Map, with obstacles extended by one cell all around.
        """
        nav_grid = np.empty((self._grid_width, self._grid_height))
        nav_grid[:] = 0       

        for x in range(1, self._grid_width - 1):
            for y in range(1, self._grid_height - 1):
                if nav_grid[x][y] == 0: # only need to set values once
                    if self.is_an_obstacle((x,y)):
                        nav_grid[x][y] = 1
                        nav_grid[x + 1][y] = 1
                        nav_grid[x + 1][y + 1] = 1
                        nav_grid[x + 1][y - 1] = 1
                        nav_grid[x][y + 1] = 1
                        nav_grid[x][y - 1] = 1
                        nav_grid[x - 1][y] = 1
                        nav_grid[x - 1][y + 1] = 1
                        nav_grid[x - 1][y - 1] = 1
                    else:
                        nav_grid[x][y] = self._grid[x][y]

        for x in range(0, self._grid_width - 1):
            nav_grid[x][0] = self._grid[x][0]
            nav_grid[x][self._grid_height - 1] = self._grid[x][self._grid_height - 1]
        for y in range(0, self._grid_height - 1):
            nav_grid[0][y] = self._grid[0][y]
            nav_grid[self._grid_width - 1][y] = self._grid[self._grid_width - 1][y]

        new_map = Map(self._x1, self._y1, self._x2, self._y2, self._scale)
        new_map._grid = nav_grid
        return new_map