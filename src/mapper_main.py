import time
import logging
from sys import argv
from multiprocessing import Queue, Process

from path_planner import PathPlanner
from mapper import LaserModel, Map, ShowMap
from controller import Controller
from goal_planner import GoalPlanner

from controller.robotrunner_v2 import goFast

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
            path, goal_point = q_showmap_path.get()
        show_map.updateMap(occupancy_map.grid, laser_model.p_max, robot_cell[0], robot_cell[1], goal_point, path)

def path_planner_job(q_path_in, q_showmap_path):
    while True:
        occupancy_map, robot_cell = q_path_in.get()
        while not q_path_in.empty():
            occupancy_map, robot_cell = q_path_in.get()

        planner = GoalPlanner(occupancy_map)
        goal_point = (0, 0)
        p = planner.closest_frontier_centroid(robot_cell)
        if p is not None:
            goal_point = p

        if goal_point:
            path_planner = PathPlanner(occupancy_map)#PathPlanner(Map.expanded_obstacles_map(occupancy_map))
            path = path_planner.get_path(robot_cell, goal_point)
            if len(path) <= 1:
                continue
            logger.info("GOAL POINT: " + str(goal_point))
            x0, y0 = path[1]
            x, y = path[-1]
            logger.info("ROBOT POS: " + str(robot_cell))
            logger.info("FIRST PATH NODE: " + str(occupancy_map.convert_to_grid_indexes(x0, y0)))
            logger.info("LAST PATH NODE: " + str(occupancy_map.convert_to_grid_indexes(x, y)))

            # For compatibility with the pure pursuit implementation
            new_path = []
            for x, y in path[1:]:
                new_node = {}
                new_node['Pose'] = {}
                new_node['Pose']['Position'] = {}
                new_node['Pose']['Position']['X'] = x
                new_node['Pose']['Position']['Y'] = y
                new_path.append(new_node)

            # Send the path to the gui
            if q_showmap_path.empty():
                coord_path = []
                for x, y in path:
                    coord_path.append(occupancy_map.convert_to_grid_indexes(x, y))
                q_showmap_path.put([coord_path, goal_point])

            goFast(new_path)

if __name__ == '__main__':
    scale = 2
    laser_max_distance = 100

    # TODO: parse arguments and print usage
    if len(argv) == 6:
        mrds_url = argv[1]
        x1 = int(argv[2])
        y1 = int(argv[3])
        x2 = int(argv[4])
        y2 = int(argv[5])
        width = x2 - x1
        height = y2 - y1
    else:
        print("Usage: mapper url x1 y1 x2 y2")
        exit()

    controller = Controller(mrds_url=mrds_url)

    laser_angles = controller.get_laser_scan_angles()
    laser_model = LaserModel(laser_angles, laser_max_distance)

    occupancy_map = Map(x1, y1, x2, y2, scale)

    pos, rot = controller.get_pos_and_orientation()
    robot_cell = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)

    q_sm = Queue()
    q_path_in = Queue()
    q_path_out = Queue()
    q_showmap_path = Queue()

    p1 = Process(target=show_map_job, args=(q_sm, scale * width, scale * height, q_showmap_path,))
    p1.daemon = False
    p1.start()
    p2 = Process(target=path_planner_job, args=(q_path_in, q_showmap_path,))
    p2.daemon = False
    p2.start()

    goal_point = (0, 0)
    while True:#TODO: while xhere are unknown cells on the map/no goals?
        laser_scan = controller.get_laser_scan()
        pos, rot = controller.get_pos_and_orientation()
        laser_model.apply_model(occupancy_map, pos, rot, laser_scan)
        robot_cell = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)

        q_sm.put([occupancy_map, laser_model, robot_cell])
        while not q_path_in.empty():
            q_path_in.get()
        q_path_in.put([occupancy_map, robot_cell])

        time.sleep(0.1)

    p.join()

