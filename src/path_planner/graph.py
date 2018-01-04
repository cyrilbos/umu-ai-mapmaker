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

        logger.debug("Constructed path planner graph with a total of {} cells and {} links".format(
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

    def a_star(self, start, goal):
        """
            Algorithm presented in both AI courses at Umu.
            Uses the euclidian distance with cspace grid indexes to compute the heuristic to the goal node.
            The heuristic from the start node computes the sum of this distance between each node from the current
            path.
        """
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

        # this dict stores for each neighbour link the node it was added from
        came_from_previous = {}
        depth = 0
        current = start
        # cameFrom is empty, so can't use get_evaluated_neighbours()
        neighbours = sorted(
            [(neighbour, heuristic(neighbour)) for neighbour in self.get_non_obstacle_neighbours(start)],
            key=itemgetter(1))
        while len(neighbours) > 0:
            previous = current
            #the expanded node is the one of lowest score
            current = neighbours.pop(0)[0]
            depth+=1
            # went outside the known map, so use the previous as a subgoal
            if self._map.is_unexplored(current):
                return construct_path_to(previous)

            came_from_previous[current] = previous

            if current == goal or depth > 255:
                return construct_path_to(current)



            neighbours += get_evaluated_neighbours(current)
            # need to sort the whole list again as new neighbours might have a lower score than previously added nodes
            neighbours = sorted(neighbours, key=itemgetter(1))

    def to_coordinates_path(self, graph_path):
        """
        Converts the path given in cspace grid indexes to a path in world position coordinates.
        """
        return [self._map.convert_to_real_position(*waypoint) for waypoint in graph_path]
