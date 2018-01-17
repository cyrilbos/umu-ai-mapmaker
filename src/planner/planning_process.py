from random import randint
from planner import PathPlanner, GoalPlanner

from logging import getLogger

logger = getLogger(__name__)


def planning_job(controller, q_path_in, q_showmap_path, scale, width, height, sound):
    path_planner = PathPlanner(8 * scale * (width + height))  # this is a limit in the depth of the A* search

    planned_goal_points = {}
    banned_goal_points = set()
    visited_goal_points = set()

    last_goal_attempts = 0
    while True:
        occupancy_map, robot_cell = q_path_in.get()
        while not q_path_in.empty():
            occupancy_map, robot_cell = q_path_in.get()

        logger.info("Finding goal point")

        navigation_map = occupancy_map.obstacle_expanded_map().obstacle_expanded_map()

        #This avoids the problem where the robot thinks it is on an obstacle, usually because of the expansion
        nearest_empty = navigation_map.get_nearest_empty_cell(robot_cell, 0)
        if nearest_empty:
            robot_cell = nearest_empty
        logger.info("yolo")
        planner = GoalPlanner(navigation_map)
        goal_point = robot_cell
        frontiers = planner.find_frontiers(navigation_map, robot_cell)
        #No more frontiers, so area fully explored
        if frontiers is None:
            logger.info("No more frontier points. Area fully explored")
            exit()

        f = planner.get_min_frontier(frontiers, robot_cell)
        p = planner.get_closest_centroid(f, robot_cell)

        # If the generated goal has already been visited, or banned, we change the frontier
        while (p is None or p in banned_goal_points or p in visited_goal_points) and len(frontiers) > 0:
            logger.info("Selected frontier already visited or banned, choosing another one")
            if f in frontiers:
                frontiers.remove(f)
            f = planner.get_min_frontier(frontiers, robot_cell)
            p = planner.get_closest_centroid(f, robot_cell)
            logger.info(frontiers)
            logger.info(f)
            logger.info(p)

        if p is not None and occupancy_map.is_in_bounds(p):
            goal_point = p
        elif p is None:
            logger.info("No more frontier points that weren't banned or visited. Area fully explored")
            exit()

        #Add new goal point to the map
        q_showmap_path.put([None, goal_point])

        if p is not None and goal_point:
            logger.info("Calculating path to goal {}".format(goal_point))

            if goal_point in planned_goal_points:
                if planned_goal_points[goal_point] >= 2:
                    logger.info("Could not find a non blocking path with extended obstacles {} times, "
                                "banning the goal {}".format(last_goal_attempts, goal_point))
                    planned_goal_points[goal_point] = None
                    banned_goal_points.add(goal_point)
                    continue
                else:
                    # Expand the map obstacles even more and try again
                    for i in range(0, planned_goal_points[goal_point]):
                        navigation_map = navigation_map.obstacle_expanded_map()
                logger.info("Could not find a path last time, retrying with extended obstacles")
                planned_goal_points[goal_point] += 1

            path = path_planner.astar(navigation_map, robot_cell, goal_point)
            if not path:
                if controller.is_blocked(1):
                    logger.info("Could not find a path and controller blocked, trying to unblock")
                    controller.unblock(1)
                else:
                    banned_goal_points.add(goal_point)
                    logger.info("Could not find a path and controller not blocked, banning goal")
                continue
            logger.info("New path found")

            if goal_point not in planned_goal_points:
                planned_goal_points[goal_point] = 0

            new_path = []
            for xg, yg in path:
                x, y = occupancy_map.center_of_cell(xg, yg)
                new_node = {'Pose': {}}
                new_node['Pose']['Position'] = {}
                new_node['Pose']['Position']['X'] = x
                new_node['Pose']['Position']['Y'] = y
                new_path.append(new_node)
            new_path.reverse()

            q_showmap_path.put([path, goal_point])

            logger.info("Following path using pure pursuit")
            if not controller.go_fast(new_path, sound):
                visited_goal_points.add(goal_point)
                logger.info("Reached goal {}".format(goal_point))
            else:
                logger.info("Got stuck while moving to goal {}, replanning".format(goal_point))
                distance_to_goal = controller.get_distance(controller.getPose()['Pose']['Position'],
                                                           new_path[len(path) - 1]['Pose']['Position'])
                logger.info("Distance to goal: {}".format(distance_to_goal))
                if distance_to_goal < 2:
                    if goal_point in planned_goal_points:
                        planned_goal_points[goal_point] = None
                    visited_goal_points.add(goal_point)

                    logger.info("Robot was close enough to the goal, so marked it as visited")
                else:
                    controller.stop()
                    controller.unblock(2)
                    logger.info("Obstacle found on the path, replanning")