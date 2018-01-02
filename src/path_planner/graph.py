from  collections import deque

import math

from logging import getLogger
from operator import itemgetter

logger = getLogger(__name__)


class Graph:
    """
        Graph implementation inspired by https://www.python.org/doc/essays/graphs/, except using
        the map (row, column) as dictionary keys.
    """
    # static table used to iterate over neighbours
    directions = (-1, 0, 1)

    def __init__(self, map):
        self._map = map
        self._grid = map.grid
        self._neighbours = {}
        link_total = 0

        # construct the graph
        for row in range(0, self._map.grid_height):
            for col in range(0, self._map.grid_width):
                cell = (row, col)
                self.construct_neighbours(cell)
                link_total += len(self._neighbours[cell])

        logger.info("Constructed path planner graph with a total of {} cells and {} links".format(
            self._map.grid_width * self._map.grid_height, link_total))

    def construct_neighbours(self, node):
        """
            Constructs the list of neighbours of node.
        """
        self._neighbours[node] = []
        for row_direction in self.directions:
            for col_direction in self.directions:
                if not (row_direction == 0 and col_direction == 0):
                    neighbour = (node[0] + row_direction, node[1] + col_direction)
                    if self._map.is_in_bounds(neighbour):
                        self._neighbours[node].append((neighbour[0], neighbour[1]))

    def get_neighbours(self, node):
        return self._neighbours[node]

    def get_non_obstacle_neighbours(self, node):
        return [n for n in self._neighbours[node] if not self._map.is_an_obstacle(n)]

    def find_path(self, start, end, path=[]):
        path = path + [start]
        if start == end:
            return path
        if not self._neighbours[start]:
            return None
        for node in self._neighbours[start]:
            if node not in path:
                newpath = self.find_path(node, end, path)
                if newpath: return newpath
        return None

    def find_shortest_path(self, start, end, path=[]):
        path = path + [start]
        if start == end:
            return path
        if not self._neighbours[start]:
            return None
        shortest = None
        for node in self._neighbours[start]:
            if node not in path:
                newpath = self.find_shortest_path(node, end, path)
                if newpath:
                    if not shortest or len(newpath) < len(shortest):
                        shortest = newpath
        return shortest

    def my_a_star(self, start, goal):
        def distance(n1, n2):
            return math.sqrt(math.pow(n1[0] - n2[0], 2) + math.pow(n1[1] - n2[1], 2))

        def start_heuristic(node):
            path = construct_path_to(node)
            cost = 0
            for i in range(0, len(path) - 1):
                cost += distance(path[i], path[i + 1])
            return cost

        def goal_heuristic(node):
            return distance(goal, node)

        def heuristic(node):
            return start_heuristic(node) + goal_heuristic(node)

        def get_evaluated_neighbours(node):
            return [(neighbour, heuristic(neighbour)) for neighbour in self.get_non_obstacle_neighbours(node) if
                    neighbour != came_from_previous[node]]

        def construct_path_to(goal):
            path = deque()
            path.append(goal)
            node = goal
            while node in came_from_previous:
                node = came_from_previous[node]
                path.appendleft(node)
            return path

        # store for each neighbour link the node it was added from
        came_from_previous = {}

        current = start
        # cameFrome is empty, so can't use get_evaluated_neighbours()
        neighbours = sorted(
            [(neighbour, heuristic(neighbour)) for neighbour in self.get_non_obstacle_neighbours(start)],
            key=itemgetter(1))
        while len(neighbours) > 0:
            previous = current
            current = neighbours.pop(0)[0]

            came_from_previous[current] = previous

            if current == goal:
                return construct_path_to(current)

            neighbours += get_evaluated_neighbours(current)
            # need to sort the whole list again as new neighbours might have a lower score than previously added nodes
            neighbours = sorted(neighbours, key=itemgetter(1))

    def to_coordinates_path(self, graph_path):
        return [self._map.convert_to_world_position(waypoint) for waypoint in graph_path]
