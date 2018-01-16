from heapq import heappush, heappop

from logging import getLogger

logger = getLogger(__name__)

class PathPlanner:
    """
    The path planner, resposible for creating a path to the goal set by the planner using A* search.
    """

    # static table used to iterate over neighbours
    neighbour_directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    def __init__(self, max_depth=200):
        self.__max_depth = 200

    def astar(self, map, start, goal):
        """
        Implementation of astar inspired by code.activestate.com/recipes/578919-python-a-pathfinding-with-binary-heap/,
        with a couple modifications # TODO: explain
        """

        def heuristic_distance(n1, n2):
            return (n2[0] - n1[0]) ** 2 + (n2[1] - n1[1]) ** 2

        closed_nodes_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: heuristic_distance(start, goal)}
        opened_nodes_heap = []
        depth = 1
        last_unexplored = start
        heappush(opened_nodes_heap, (fscore[start], start))  # Using a binary heap makes the list of nodes sorted

        while opened_nodes_heap:

            current = heappop(opened_nodes_heap)[1]
            if current == goal:
                logger.info("Goal found, returning path")
                return self.construct_path(came_from, current)
            if map.is_unexplored(current):
                last_unexplored = current

            depth += 1
            if depth > self.__max_depth:
                logger.info("Max depth reached, returning path to closest knownww subgoal")
                return self.construct_path(came_from, last_unexplored)

            closed_nodes_set.add(current)
            for i, j in PathPlanner.neighbour_directions:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + heuristic_distance(current, neighbor)
                if 0 <= neighbor[0] < map.grid_width:
                    if 0 <= neighbor[1] < map.grid_height:
                        if map.is_an_obstacle(neighbor):
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue

                if neighbor in closed_nodes_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                # if the neighbor is not in opened nodes or its score is better, store the path and scores
                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in opened_nodes_heap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic_distance(neighbor, goal)
                    heappush(opened_nodes_heap, (fscore[neighbor], neighbor))

        return None

    def construct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        return path