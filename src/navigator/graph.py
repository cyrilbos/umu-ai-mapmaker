class Graph:
    """
        Graph implementation inspired by https://www.python.org/doc/essays/graphs/, except using
        the map (row, column) as dictionary keys.
    """
    #static table used to iterate over neighbours
    directions = (0, -1, 1)

    def __init__(self, grid):
        self._grid = grid
        self._data = {}
        for row in grid:
            for col in row:
                self.apply_to_neighbours(row, col, self.__construct_links())

    def __construct_links(self, row, col, neighb_row, neighb_col):
        try:
            self._data[(row, col)].append(neighb_col, neighb_row)
        except KeyError:
            self._data[(row, col)] = []


    def apply_to_neighbours(self, row, col, function):
        for row_direction in self.directions:
            for col_direction in self.directions:
                # optimization to avoid one if test every iteration. see
                #avoids if col_direction != 0 and row_direction != 0
                self.apply_to_neighbours = self._skip_first_apply_to_neighbours

    def _skip_first_apply_to_neighbours(self, row, col, function):
        """
            Applies function to every neighbour around node (row, col),
            passing arguments in the order row, col, neighbour_row, neighbour_col
        """
        for row_direction in self.directions:
            for col_direction in self.directions:
                cur_row = row + row_direction
                cur_col = col + col_direction
                if self._grid.is_in_bounds((row, col)):
                    function(row, col, cur_row, cur_col)

    def find_path(self, start, end, path=[]):
        path = path + [start]
        if start == end:
            return path
        if not self._data.has_key(start):
            return None
        for node in self[start]:
            if node not in path:
                newpath = self.__find_path(self, node, end, path)
                if newpath: return newpath
        return None

    def find_shortest_path(graph, start, end, path=[]):
        path = path + [start]
        if start == end:
            return path
        if not graph.has_key(start):
            return None
        shortest = None
        for node in graph[start]:
            if node not in path:
                newpath = find_shortest_path(graph, node, end, path)
                if newpath:
                    if not shortest or len(newpath) < len(shortest):
                        shortest = newpath
        return shortest

    def to_coordinates_path(self, graph_path):
        return [self._grid.convert_to_real_position(waypoint) for waypoint in graph_path]



