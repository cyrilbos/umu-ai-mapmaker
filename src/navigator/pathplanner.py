from navigator.graph import Graph


class PathPlanner:
    """
    The path planner, resposible for creating a path to the
    goal set by the planner.

    Wayfront or A*?

    Input: a goal position / grid coordinate?

    Output: a path to the goal, avoiding obstacles?
    """
    pass

    def __init__(self, grid):
        self._grid = grid
        self._graph = Graph(grid)

    def get_path(self, robot_position, target_position):
        """
            Returns a path to target_position from robot_position, avoiding known obstacles.
            :rtype List[float]
        """
        path = []

        robot_cell = self._grid.convert_to_grid_indexes(robot_position.x, robot_position.y)
        target_cell = self._grid.convert_to_grid_indexes(target_position.x, target_position.y)

        self._grid.find_path(robot_cell, target_cell)

        return path