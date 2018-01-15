import time
import logging
from sys import argv
from multiprocessing import Queue, Process
from heapq import *

from path_planner import PathPlanner
from mapper import LaserModel, Map, ShowMap
from controller import Controller
from goal_planner import GoalPlanner

from controller.robotrunner_v2 import goFast
from controller.robotrunner_v2 import reposition

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

def planning_job(q_path_in, q_showmap_path, q_path_out, mrds_url, starting_pos, sound):
    path_planner = PathPlanner()
    banned_goal_points = []
    visited_goal_points = []
    while True:
        occupancy_map, robot_cell = q_path_in.get()
        while not q_path_in.empty():
            occupancy_map, robot_cell = q_path_in.get()

        else:
            logger.info("Finding goal point")
            navigation_map = occupancy_map.obstacle_extended_map().obstacle_extended_map()

            planner = GoalPlanner(navigation_map)
            goal_point = starting_pos
            p = planner.closest_frontier_centroid(robot_cell)
            blocking_goal_search_count = 0
            while p in banned_goal_points and blocking_goal_search_count < 10:
                navigation_map = navigation_map.obstacle_extended_map()
                planner = GoalPlanner(navigation_map)
                p = planner.closest_frontier_centroid(robot_cell)
                blocking_goal_search_count += 1
            if blocking_goal_search_count >= 10:
                exit()

            if p is not None and occupancy_map.is_in_bounds(goal_point):
                goal_point = p

            q_showmap_path.put([None, goal_point])
            q_path_out.put(goal_point)

            if goal_point:
                logger.info("Calculating path")
                path = path_planner.astar(navigation_map.grid, robot_cell, goal_point)

                if not path:
                    # Try again with expanded obstacles
                    navigation_map = navigation_map.obstacle_extended_map()
                    path = path_planner.astar(navigation_map.grid, robot_cell, goal_point)
                if not path:
                    # Probably stuck "inside" an obstacle, try to get out
                    logger.info("Can not find a path to next goal, trying to get out of obstacle")
                    reposition(mrds_url)
                    banned_goal_points.append(goal_point)
                    continue
                #if len(path) <= 1:
                #    continue
                # For compatibility with the pure pursuit implementation
                logger.info("new path found")
                new_path = []
                for xg, yg in path:
                    if navigation_map.is_an_obstacle((xg, yg)):
                        logger.error("one of the path node is an obstacle in navigation map")
                        if occupancy_map.is_an_obstacle((xg, yg)):
                            logger.error("one of the path node is an obstacle in occupancy map")

                    x, y = occupancy_map.center_of_cell(xg, yg)
                    new_node = {}
                    new_node['Pose'] = {}
                    new_node['Pose']['Position'] = {}
                    new_node['Pose']['Position']['X'] = x
                    new_node['Pose']['Position']['Y'] = y
                    new_path.append(new_node)
                new_path.reverse()

                q_showmap_path.put([path, goal_point])

                logger.info("Following path using pure pursuit")
                goFast(new_path, mrds_url, sound)
                visited_goal_points.append(goal_point)


if __name__ == '__main__':
    scale = 2 #resolution, i.e number of cells in the cspace grid for each meter
    laser_max_distance = 40

    if len(argv) == 6:
        mrds_url = argv[1]
        x1 = int(argv[2])
        y1 = int(argv[3])
        x2 = int(argv[4])
        y2 = int(argv[5])
        width = x2 - x1
        height = y2 - y1
        sound = False
    else:
        #print("Usage: python3 mapper_main.py url x1 y1 x2 y2")
        #exit()
        # Temporary debug settings
        mrds_url = "localhost:50000"
        x1 = -70
        y1 = -20
        x2 = 70
        y2 = 70
        width = x2 - x1
        height = y2 - y1
        sound = True

    controller = Controller(mrds_url=mrds_url)

    laser_angles = controller.get_laser_scan_angles()
    laser_model = LaserModel(laser_angles, laser_max_distance)

    occupancy_map = Map(x1, y1, x2, y2, scale)

    pos, rot = controller.get_pos_and_orientation()
    robot_cell = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)
    starting_pos = robot_cell

    q_sm = Queue()
    q_path_in = Queue()
    q_path_out = Queue()
    q_showmap_path = Queue()

    p1 = Process(target=show_map_job, args=(q_sm, scale * width, scale * height, q_showmap_path,))
    p1.daemon = False
    p1.start()
    p2 = Process(target=planning_job, args=(q_path_in, q_showmap_path, q_path_out, mrds_url, starting_pos, sound,))
    p2.daemon = False
    p2.start()

    no_goal_found = 0

    goal_point = (0, 0)
    while p2.is_alive():
        laser_scan = controller.get_laser_scan()
        pos, rot = controller.get_pos_and_orientation()
        laser_model.apply_model(occupancy_map, pos, rot, laser_scan)
        robot_cell = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)

        if q_sm.empty():
            q_sm.put([occupancy_map, laser_model, robot_cell])
        while not q_path_in.empty():
            q_path_in.get()
        q_path_in.put([occupancy_map, robot_cell])

        if q_path_out.empty():
            new_goal_available = False
        else:
            new_goal_available = True
            while not q_path_out.empty():
                goal_point = q_path_out.get()

        logger.debug("goal {}".format(goal_point))
        logger.debug("robot cell {}".format(robot_cell))
        if new_goal_available:
            if goal_point == robot_cell:
                logger.info("no_goal_found {}".format(no_goal_found))

                no_goal_found += 1
                if no_goal_found >= 5:
                    break
            else:
                no_goal_found = 0

        time.sleep(0.075)

    p1.join()
    p2.join()

