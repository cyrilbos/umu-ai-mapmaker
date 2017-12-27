from collections import deque

OPEN_MAX_VALUE = 1

class Planner:
    """
    Responsible for choosing the next area to explore.

    Frontier-based exploration?

    Input: a CSpace grid produced by the mapper?

    Output: A goal position (or grid coordinate) to explore?
    """


    def __init__(self):
        pass

    def find_frontiers(cspace_map, robot_pos):
        # Algorithm from http://www.ifaamas.org/Proceedings/aamas2012/papers/3A_3.pdf

        grid = cspace_map.grid()
        robot_position = (robot_pos.x, robot_pos.y)

        frontiers = []
        queue_m = deque([])
        map_open = set([])
        map_closed = set([])
        frontier_open = set([])
        frontier_closed = set([])
        map_open_space = find_open(cspace_map)

        queue_m.append(robot_position)
        map_open.add(robot_position)

        while queue_m:
            p = queue_m.popleft()

            if p in map_closed:
                continue

            if is_frontier_point(p):
                queue_f = dequeue([])
                new_frontier = set([])

                queue_f.append(p)
                frontier_open.append(p)

                while queue_f:
                    q = queue_f.popleft()

                    if q in map_closed and q in frontier_closed:
                        continue

                    if is_frontier_point(q):
                        new_frontier.add(q)
                        for w in adjacent(q):
                            if (w not in frontier_open and w not in frontier_closed and
                                    w not in map_closed):
                                queue_f.append(w)
                                fronter_open.add(w)

                    frontier_closed.add(q)

                frontiers.append(new_frontier)
                for pt in new_frontier:
                    map_closed.add(pt)
            
            for v in adjacent(p):
                if (v not in map_open and v not in map_closed and
                        has_neighbor_in(v, map_open_space)):
                    queue_m.append(v)
                    map_open.add(v)

            map_close.add(p)

    def find_open(self, cspace_map):
        """
        Returns the "open" positions in the grid.
        """
        map_open_space = set([])
        grid = cspace_map.grid()

        for x in range(cspace_map.grid_width()):
            for y in range(cspace_map.grid_height()):
                if grid[x][y] <= OPEN_MAX_VALUE:
                    map_open_space.add((x, y))

        return map_open_space

    def has_neighbor_in(self, point, collection):
        """
        Returns True if point has a neighboring position in collection.
        """
        x, y = point
        if (x + 1, y) in collection:
            return True
        if (x, y + 1) in collection:
            return True
        if (x + 1, y + 1) in collection:
            return True
        if (x - 1, y) in collection:
            return True
        if (x, y - 1) in collection:
            return True;
        if (x - 1, y - 1) in collection:
            return True
        if (x + 1, y - 1) in collection:
            return True
        if (x - 1, y + 1) in collection:
            return True
        return False

    def adjacent(self, point):
        """
        Returns the positions adjacent to the point.
        """
        x, y = point
        adjacent_points = set([])
        adjacent_points.add((x + 1, y))
        adjacent_points.add((x, y + 1))
        adjacent_points.add((x + 1, y + 1))
        adjacent_points.add((x - 1, y))
        adjacent_points.add((x, y - 1))
        adjacent_points.add((x - 1, y - 1))
        adjacent_points.add((x + 1, y - 1))
        adjacent_points.add((x - 1, y + 1))
        return adjacent_points

    def is_frontier_point(self, point, cspace_map):
        # Untested!
        grid = cspace_map.grid()

        x, y = point

        if cspace_map[x][y] != 0.5: # float comparison, does this work?
            return False

        for p in adjacent(point):
            x, y = p
            if grid[x, y] <= OPEN_MAX_VALUE:
                return True

        return False
