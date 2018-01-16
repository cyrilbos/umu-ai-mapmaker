import time
import logging
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
        time.sleep(0.2)

def planning_job(controller, q_path_in, q_showmap_path, q_path_out, sound):
    path_planner = PathPlanner()
    banned_goal_points = set()
    visited_goal_points = set()
    planned_goal_points = set()
    open_max_value = 0.1
    max_attempts = 10

    last_goal = None
    last_goal_attempts = 0
    while True:
        occupancy_map, robot_cell = q_path_in.get()
        while not q_path_in.empty():
            occupancy_map, robot_cell = q_path_in.get()

        logger.info("Finding goal point")
        navigation_map = occupancy_map.obstacle_expanded_map() #TODO: try with forced one time expanded obstacles

        planner = GoalPlanner(navigation_map, open_max_value)
        goal_point = robot_cell
        p = planner.closest_frontier_centroid(robot_cell)
        blocking_goal_search_count = 0
        # Check if goal point is banned, i.e we could not find a path to it.
        # If it is banned, increase the open_max_value parameter value and try again.
        # There is an arbitrarily defined trial limit to consider the area was fully explored,
        # or that possible goals do not have possible paths.
        while p in banned_goal_points and blocking_goal_search_count < max_attempts:
            open_max_value *= 1.2
            planner = GoalPlanner(navigation_map, open_max_value)
            p = planner.closest_frontier_centroid(robot_cell)
            blocking_goal_search_count += 1

        if blocking_goal_search_count >= max_attempts:
            logger.info("Found banned goal {} times in a row. Area completely explored".format(max_attempts))
            exit()
        open_max_value = 0.1
        already_visited_goal_count = 0

        # Check in a similar way if the goal point is already visited,
        # if it is then it is uninteresting to go there again.
        while p is None and goal_point in visited_goal_points and already_visited_goal_count < max_attempts:
            open_max_value *= 1.2
            planner = GoalPlanner(navigation_map, open_max_value)
            p = planner.closest_frontier_centroid(robot_cell)
            already_visited_goal_count += 1

        if already_visited_goal_count >= max_attempts:
            logger.info("Found already visited goal {} times in a row. Area completely explored".format(max_attempts))
            exit()

        if p is not None and occupancy_map.is_in_bounds(p):
            goal_point = p

        q_showmap_path.put([None, goal_point])
        q_path_out.put(goal_point)

        if goal_point:
            logger.info("Calculating path to goal {}".format(goal_point))
            if goal_point in planned_goal_points:
                if last_goal == goal_point:
                    last_goal_attempts += 1
                    if last_goal_attempts >= 5:
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
            path = path_planner.astar(navigation_map, robot_cell, goal_point)
            if not path:
                # Probably stuck "inside" an obstacle
                controller.unblock(None, 1)
                logger.info("Could not find a path, trying to unblock from obstacle")
                #banned_goal_points.add(goal_point)
                #logger.info("Could not find a path, banned blocking goal {}".format(goal_point))
                continue

            # For compatibility with the pure pursuit implementation
            logger.info("New path found")
            planned_goal_points.add(goal_point)
            last_goal = goal_point

            new_path = []
            for xg, yg in path:
                if navigation_map.is_an_obstacle((xg, yg)):
                    logger.error("one of the path node is an obstacle in navigation map")
                    if occupancy_map.is_an_obstacle((xg, yg)):
                        logger.error("one of the path node is an obstacle in occupancy map")

                x, y = occupancy_map.center_of_cell(xg, yg)
                new_node = {'Pose': {}}
                new_node['Pose']['Position'] = {}
                new_node['Pose']['Position']['X'] = x
                new_node['Pose']['Position']['Y'] = y
                new_path.append(new_node)
            new_path.reverse()

            q_showmap_path.put([path, goal_point])

            logger.info("Following path using pure pursuit")
            if controller.go_fast(new_path, sound):
                visited_goal_points.add(goal_point)
                logger.info("Reached goal {}".format(goal_point))
            else:
                logger.info("Got stuck while moving to goal {}, replanning".format(goal_point))


if __name__ == '__main__':
    scale = 2  # resolution, i.e number of cells in the cspace grid for each meter
    laser_max_distance = 40

    if len(argv) == 6:
        mrds_url = argv[1]
        x1 = int(argv[2])
        y1 = int(argv[3])
        x2 = int(argv[4])
        y2 = int(argv[5])
        width = x2 - x1
        height = y2 - y1
        sound = False #can be set to True to play a beep when the reactive stop activates
    else:
        #print("Usage: python3 mapper_main.py url x1 y1 x2 y2")
        #exit()
        # Temporary debug settings
        mrds_url = "localhost:50000"
        x1 = -70
        y1 = -70
        x2 = 70
        y2 = 70
        width = x2 - x1
        height = y2 - y1
        sound = True

    controller = Controller(mrds_url=mrds_url)

    laser_angles = controller.get_laser_scan_angles()
    laser_model = LaserModel(laser_angles, laser_max_distance)

    occupancy_map = Map(x1, y1, x2, y2, scale)

    q_sm = Queue()
    q_path_in = Queue()
    q_path_out = Queue()
    q_showmap_path = Queue()

    show_map_process = Process(target=show_map_job, args=(q_sm, scale * width, scale * height, q_showmap_path,))
    show_map_process.daemon = False
    show_map_process.start()

    planning_process = Process(target=planning_job, args=(controller, q_path_in, q_showmap_path, q_path_out, sound))
    planning_process.daemon = True
    planning_process.start()

    no_goal_found = 0

    goal_point = (0, 0)
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
