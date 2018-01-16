import time
import logging
from random import randint
from sys import argv
from multiprocessing import Queue, Process

from planner import PathPlanner
from mapper import LaserModel, Map, ShowMap
from controller import Controller
from planner import GoalPlanner

logging.basicConfig(format="[%(asctime)s %(filename)s:%(lineno)s - %(funcName)20s() ] %(message)s",
                    level=logging.INFO)
logger = logging.getLogger(__name__)


def show_map_job(q_sm, width, height, q_showmap_path):
    show_map = ShowMap(width, height, True)  # rows, cols, showgui
    show_expanded_map = ShowMap(width, height, True, name="Expanded map")  # rows, cols, showgui
    goal_point = (0, 0)
    path = None
    while True:
        occupancy_map, laser_model, robot_cell = q_sm.get()
        while not q_sm.empty():
            occupancy_map, laser_model, robot_cell = q_sm.get()
        if not q_showmap_path.empty():
            while not q_showmap_path.empty():
                path, goal_point = q_showmap_path.get()
        show_map.updateMap(occupancy_map.grid, laser_model.p_max, robot_cell[0], robot_cell[1], goal_point, path)
        show_expanded_map.updateMap(occupancy_map.obstacle_expanded_map().obstacle_expanded_map().grid, laser_model.p_max, robot_cell[0], robot_cell[1], goal_point, path)
        time.sleep(0.5)


def planning_job(controller, q_path_in, q_showmap_path, q_path_out, scale, width, height, sound):
    path_planner = PathPlanner(2 * scale * (width + height))

    blocking_goal_points = set()
    banned_goal_points = set()
    visited_goal_points = set()

    max_attempts = 10

    last_goal = None
    last_goal_attempts = 0
    while True:
        occupancy_map, robot_cell = q_path_in.get()
        while not q_path_in.empty():
            occupancy_map, robot_cell = q_path_in.get()

        logger.info("Finding goal point")

        navigation_map = occupancy_map.obstacle_expanded_map().obstacle_expanded_map().obstacle_expanded_map()   # TODO: try with forced one time expanded obstacles
        nearest_empty = navigation_map.get_nearest_empty_cell(robot_cell, 0)
        if nearest_empty:
            robot_cell = nearest_empty
        planner = GoalPlanner(navigation_map)
        goal_point = robot_cell
        frontiers = planner.closest_frontier_centroid(robot_cell)
        if frontiers is None:
            exit()
        p = planner.get_min_frontier(frontiers[randint(0, len(frontiers) - 1)], robot_cell)

        # Check if goal point is blocking, i.e we could not find a path to it.
        # If it is banned, increase the open_max_value parameter value and try again.
        # There is an arbitrarily defined trial limit to consider the area was fully explored,
        # or that possible goals do not have possible paths.
        """
        blocking_goal_search_count = 0
        while p in blocking_goal_points and blocking_goal_search_count < max_attempts:
            planner = GoalPlanner(navigation_map, open_max_value)
            p = planner.closest_frontier_centroid(robot_cell)
            path = path_planner.astar(navigation_map, robot_cell, p)
            blocking_goal_search_count += 1
            if not path and blocking_goal_search_count >= max_attempts:
                logger.info("Found blocking goal {} times in a row. Banning goal".format(max_attempts))
                blocking_goal_points.remove(p)
                banned_goal_points.add(p)
                continue
            open_max_value *= 2
        open_max_value = starting_open_max_value
        """
        already_visited_goal_count = 0

        # Check in a similar way if the goal point is already visited,
        # if it is then it is uninteresting to go there again.
        while (p is None or p in banned_goal_points) and already_visited_goal_count < max_attempts \
                and len(frontiers) > 0:
            # open_max_value *= 2
            # planner = GoalPlanner(navigation_map, open_max_value)
            if p in frontiers:
                frontiers.remove(p)
            p = planner.get_min_frontier(frontiers[randint(0, len(frontiers) - 1)],
                                         robot_cell)  # planner.closest_frontier_centroid(robot_cell))
            path = path_planner.astar(navigation_map, robot_cell, p)
            already_visited_goal_count += 1

            # if not path and already_visited_goal_count >= max_attempts:
            #    logger.info("Found already visited goals {} times in a row. Area completely explored".format(max_attempts))
            #    exit()

        if p is not None and occupancy_map.is_in_bounds(p):
            goal_point = p

        q_showmap_path.put([None, goal_point])
        q_path_out.put(goal_point)

        if goal_point:
            logger.info("Calculating path to goal {}".format(goal_point))
            if goal_point in blocking_goal_points:
                if last_goal == goal_point:
                    last_goal_attempts += 1
                    if last_goal_attempts >= 2:
                        logger.info("Could not find a non blocking path with extended obstacles {} times, "
                                    "banning the goal {}".format(last_goal_attempts, goal_point))
                        banned_goal_points.add(goal_point)
                        continue
                else:
                    last_goal_attempts = 0
                i = 0
                while i < last_goal_attempts:
                    navigation_map = navigation_map.obstacle_expanded_map()
                    i += 1
                # Try again with expanded obstacles
                logger.info("Could not find a path last time, retrying with extended obstacles")
            path = path_planner.astar(navigation_map, robot_cell, goal_point)
            if not path:
                # Probably stuck "inside" an obstacle
                if goal_point != last_goal:
                    controller.unblock(None, 1)
                logger.info("Could not find a path to {}, trying to unblock from obstacle".format(goal_point))
                banned_goal_points.add(goal_point)
                continue
            logger.info("New path found")
            # planned_goal_points.add(goal_point)
            last_goal = goal_point

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
                    visited_goal_points.add(goal_point)
                    banned_goal_points.add(goal_point)

                    logger.info("Robot was close enough to the goal, so marked it as visited")
                else:
                    controller.unblock(None, 1)
                    # occupancy_map, robot_cell = q_path_in.get()
                    # for xg, yg in path:
                    #    if occupancy_map.is_an_obstacle((xg, yg)):
                    logger.info("Obstacle found on the path, adding blocking goal")
                    banned_goal_points.add(goal_point)
                    #        blocking_goal_points.add(goal_point)
                    # time.sleep(0.1) #consumed input so wait to get it againa


if __name__ == '__main__':
    scale = 8  # resolution, i.e number of cells in the cspace grid for each meter
    laser_max_distance = 40

    if len(argv) == 6:
        mrds_url = argv[1]
        x1 = int(argv[2])
        y1 = int(argv[3])
        x2 = int(argv[4])
        y2 = int(argv[5])
    else:
        # print("Usage: python3 mapper_main.py url x1 y1 x2 y2")
        # exit()
        # Temporary debug settings
        mrds_url = "localhost:50000"
        x1 = -7
        y1 = -7
        x2 = 10
        y2 = 10
    width = x2 - x1
    height = y2 - y1
    sound = False  # can be set to True to play a beep when the reactive stop activates

    controller = Controller(mrds_url=mrds_url, lin_spd=0.6)

    laser_angles = controller.get_laser_scan_angles()
    laser_model = LaserModel(laser_angles, laser_max_distance)

    occupancy_map = Map(x1, y1, x2, y2, scale)

    pose = controller.getPose()['Pose']
    controller.post_speed(3, 0.2)
    timer = 1
    # Initial scan to choose a better first frontier
    while timer > 0:
        laser_model.apply_model(occupancy_map, pose['Position'], pose['Orientation'], controller.get_laser_scan())
        time.sleep(0.1)
        timer -= 0.1
    controller.stop()

    q_sm = Queue()
    q_path_in = Queue()
    q_path_out = Queue()
    q_showmap_path = Queue()

    show_map_process = Process(target=show_map_job, args=(q_sm, scale * width, scale * height, q_showmap_path,))
    show_map_process.daemon = False
    show_map_process.start()

    planning_process = Process(target=planning_job,
                               args=(controller, q_path_in, q_showmap_path, q_path_out, scale, width, height, sound))
    planning_process.daemon = True
    planning_process.start()

    while planning_process.is_alive():
        laser_scan = controller.get_laser_scan()
        pose = controller.getPose()['Pose']
        pos = pose['Position']
        laser_model.apply_model(occupancy_map, pos, pose['Orientation'], laser_scan)
        robot_cell = occupancy_map.convert_to_grid_indexes(pos['X'], pos['Y'])

        if q_sm.empty():
            q_sm.put([occupancy_map, laser_model, robot_cell])
        while not q_path_in.empty():
            q_path_in.get()
        q_path_in.put([occupancy_map, robot_cell])

        time.sleep(0.05)

    show_map_process.terminate()
