from path_planner.graph import Graph


class PathPlanner:
    """
    The path planner, resposible for creating a path to the
    goal set by the planner.

    Wayfront or A*?

    Input: a goal position / grid coordinate?

    Output: a path to the goal, avoiding obstacles?
    """
    pass

    def __init__(self, map):
        self._map = map
        self._graph = Graph(map)

    def get_path(self, robot_cell, target_cell):
        """
            Returns a path to target_position from robot_position, avoiding known obstacles.
            :rtype List[float]
        """
        path = self._graph.my_a_star(robot_cell, target_cell)
        if path is not None:
            return self._graph.to_coordinates_path(path)
