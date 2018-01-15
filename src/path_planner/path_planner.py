from heapq import heappush, heappop

class PathPlanner:
    """
    The path planner, resposible for creating a path to the goal set by the planner using A* search.
    """

    # static table used to iterate over neighbours
    neighbour_directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    def astar(self, grid, start, goal):
        """
        Implementation of astar inspired by code.activestate.com/recipes/578919-python-a-pathfinding-with-binary-heap/,
        with a couple modifications # TODO: explain
        """

        def heuristic_distance(n1, n2):
            return (n2[0] - n1[0]) ** 2 + (n2[1] - n1[1]) ** 2

        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: heuristic_distance(start, goal)}
        oheap = []

        heappush(oheap, (fscore[start], start))

        while oheap:

            current = heappop(oheap)[1]

            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data

            close_set.add(current)
            for i, j in PathPlanner.neighbour_directions:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + heuristic_distance(current, neighbor)
                if 0 <= neighbor[0] < grid.shape[0]:
                    if 0 <= neighbor[1] < grid.shape[1]:
                        if grid[neighbor[0]][neighbor[1]] > 0.6:
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                # if the neighbor is not in opened nodes or its score is better, store the path and scores
                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic_distance(neighbor, goal)
                    heappush(oheap, (fscore[neighbor], neighbor))

        return False