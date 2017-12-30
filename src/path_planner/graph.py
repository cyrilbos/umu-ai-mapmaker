import sys

import math

from logging import getLogger

logger = getLogger(__name__)


class Graph:
    """
        Graph implementation inspired by https://www.python.org/doc/essays/graphs/, except using
        the map (row, column) as dictionary keys.
    """
    #static table used to iterate over neighbours
    directions = (0, -1, 1)

    def __init__(self, map):
        self._map = map
        self._grid = map.grid()
        self._data = {}
        tot = 0
        for row in range(0, len(self._grid)):
            for col in range(0, len(self._grid[row])):
                cell = (row, col)
                self._data[cell] = []
                self.apply_to_neighbours(cell, self.__construct_links, row, col)
                tot += len(self._data[cell])
        print(tot)
    def __construct_links(self, neighbour, row, col):
        self._data[(row, col)].append((neighbour[0], neighbour[1]))

    def apply_to_neighbours(self, current, func, *args):
        """
            Applies function to every neighbour around node (row, col),
            passing arguments in the order row, col, neighbour_row, neighbour_col
        """
        for row_direction in self.directions:
            for col_direction in self.directions:
                if row_direction != 0 and col_direction != 0:
                    neighbour = (current[0] + row_direction, current[1] + col_direction)
                    if self._map.is_in_bounds(neighbour):
                        func(neighbour, *args)

    def find_path(self, start, end, path=[]):
        path = path + [start]
        if start == end:
            return path
        if not self._data[start]:
            return None
        for node in self._data[start]:
            if node not in path:
                newpath = self.find_path(node, end, path)
                if newpath: return newpath
        return None

    def find_shortest_path(self, start, end, path=[]):
        path = path + [start]
        if start == end:
            return path
        if not self._data[start]:
            return None
        shortest = None
        for node in self._data[start]:
            if node not in path:
                newpath = self.find_shortest_path(node, end, path)
                if newpath:
                    if not shortest or len(newpath) < len(shortest):
                        shortest = newpath
        return shortest

    def a_star(self, start, goal):
        def distance(n1, n2):
            return math.sqrt(math.pow(n1[0] - n2[0], 2) + math.pow(n1[1] - n2[1], 2))

        def start_heuristic(node):
            return distance(node, start)

        def goal_heuristic(node):
            return distance(goal, node)

        def heuristic(node):
            return start_heuristic(node) + goal_heuristic(node)

        def treat_neighbour(neighbour):
            if neighbour in closedSet:
                return
            if neighbour not in openSet:
                tentative_gScore = gScore[current] + distance(current, neighbour)
                if tentative_gScore >= gScore[neighbour]:
                    return
                cameFrom[neighbour] = current
                gScore[neighbour] = tentative_gScore
                tentative_fScore = gScore[neighbour] + heuristic(neighbour)
                fScore[neighbour] = tentative_fScore

        def reconstruct_path(current):
            total_path = [current]
            while current in cameFrom:
                current = cameFrom[current]
                total_path.append(current)
                cameFrom[current] = None# ?
            return total_path

        closedSet = set()

        openSet = {start}

        cameFrom = {}

        gScore = {}
        for row in range(0, len(self._grid)):
            for col in range(0, len(self._grid[row])):
                gScore[(row, col)] = sys.maxsize #infinity

        gScore[start] = 0

        fScore = {}
        for row in range(0, len(self._grid)):
            for col in range(0, len(self._grid[row])):
                fScore[(row, col)] = sys.maxsize #infinity

        fScore[start] = goal_heuristic(start)

        while len(openSet) != 0:
            current = min(fScore, key=fScore.get)
            if current not in openSet:
                raise Exception("a_star iterating on a non discovered node")

            if current == goal:
                return reconstruct_path(current)

            openSet.remove(current)
            closedSet.add(current)

            logger.info("new cell {}".format(current))
            self.apply_to_neighbours(current, treat_neighbour)



    def to_coordinates_path(self, graph_path):
        return [self._grid.convert_to_real_position(waypoint) for waypoint in graph_path]



