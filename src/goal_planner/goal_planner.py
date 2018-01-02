from collections import deque
from math import hypot

OPEN_MAX_VALUE = 0.1

# This is for the apartment, the factory environment could
# probably use a much higher value
MIN_NUM_FRONTIER_POINTS = 10

class Planner:
    """
    Responsible for choosing the next area to explore.

    Frontier-based exploration?

    Input: a CSpace grid produced by the mapper?

    Output: A goal position (or grid coordinate) to explore?
    """


    def __init__(self, cspace_map):
        self._cspace_map = cspace_map

    def _distance(self, point, robot_indexes):
        x1, y1 = robot_indexes
        x2, y2 = point
        return hypot(x2 - x1, y2 - y1)

    def closest_frontier_centroid(self, robot_indexes):
        frontiers = self.find_frontiers(self._cspace_map, robot_indexes)
        if not frontiers:
            return None
        frontier_centroids = self.find_centroids(frontiers)
        return min(frontier_centroids, key=lambda p: self._distance(p, robot_indexes))

    def find_centroids(self, frontiers):
        return [self.centroid(f) for f in frontiers]

    def centroid(self, frontier):
        x_c = 0
        y_c = 0

        for p in frontier:
            x, y = p
            x_c += x
            y_c += y

        x_c = x_c // len(frontier)
        y_c = y_c // len(frontier)

        return (x_c, y_c)

    def find_frontiers(self, cspace_map, robot_indexes):
        # Algorithm from http://www.ifaamas.org/Proceedings/aamas2012/papers/3A_3.pdf

        grid = cspace_map.grid
        robot_position = (robot_indexes[0], robot_indexes[1])

        frontiers = []
        queue_m = deque([])
        map_open = set([])
        map_closed = set([])
        frontier_open = set([])
        frontier_closed = set([])

        queue_m.append(robot_position)
        map_open.add(robot_position)

        while queue_m:
            p = queue_m.popleft()

            if p in map_closed:
                continue

            if self.is_frontier_point(p, cspace_map):
                queue_f = deque([])
                new_frontier = set([])

                queue_f.append(p)
                frontier_open.add(p)

                while queue_f:
                    q = queue_f.popleft()

                    if q in map_closed and q in frontier_closed:
                        continue

                    if self.is_frontier_point(q, cspace_map):
                        new_frontier.add(q)
                        for w in self.adjacent(q):
                            if (w not in frontier_open and w not in frontier_closed and
                                    w not in map_closed):
                                queue_f.append(w)
                                frontier_open.add(w)

                    frontier_closed.add(q)

                if len(new_frontier) > MIN_NUM_FRONTIER_POINTS:
                    frontiers.append(new_frontier)
                for pt in new_frontier:
                    map_closed.add(pt)
            
            for v in self.adjacent(p):
                if (v not in map_open and v not in map_closed and
                        self.has_open_neighbor(v)):
                    # TODO rewrite this to has_neighbor_in_open_space (don't need the set).
                    queue_m.append(v)
                    map_open.add(v)

            map_closed.add(p)

        return frontiers

    def has_open_neighbor(self, point):
        """
        Returns True if point has an open neighbor
        """
        if not self._cspace_map.is_in_bounds(point):
            return False
        
        neighbors = self.adjacent(point)
        grid = self._cspace_map.grid

        for p in neighbors:
            x, y = p
            if grid[x][y] <= OPEN_MAX_VALUE:
                return True

        return False
        

    def adjacent(self, point):
        """
        Returns the positions adjacent to the point.
        """
        x, y = point

        x_max = self._cspace_map.grid_width - 1
        y_max = self._cspace_map.grid_height - 1

        adjacent_points = set([])
        
        if x < x_max:
            adjacent_points.add((x + 1, y))
        if y < y_max:
            adjacent_points.add((x, y + 1))
        if x < x_max and y < y_max:
            adjacent_points.add((x + 1, y + 1))
        if x > 0:
            adjacent_points.add((x - 1, y))
        if y > 0:
            adjacent_points.add((x, y - 1))
        if x > 0 and y > 0:
            adjacent_points.add((x - 1, y - 1))
        if x < x_max and y > 0:
            adjacent_points.add((x + 1, y - 1))
        if x > 0 and y < y_max:
            adjacent_points.add((x - 1, y + 1))

        return adjacent_points

    def is_frontier_point(self, point, cspace_map):
        grid = cspace_map.grid

        if not cspace_map.is_in_bounds(point):
            return False

        x, y = point

        epsilon = 0.1
        if abs(grid[x][y] - 0.5) > epsilon:
            return False

        for p in self.adjacent(point):
            x, y = p
            if grid[x, y] <= OPEN_MAX_VALUE:
                return True

        return False
