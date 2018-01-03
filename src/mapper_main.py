from sys import argv
import time
from multiprocessing import Queue, Process


from threading import Thread

from path_planner import PathPlanner

from mapper import LaserModel, Map, ShowMap

from controller import FixedController

from goal_planner import GoalPlanner

from c14tns_old.robotrunner_v2 import goFast

import logging


logging.basicConfig(format="[%(asctime)s %(filename)s:%(lineno)s - %(funcName)20s() ] %(message)s",
                    level=logging.INFO)

logger = logging.getLogger(__name__)


def planner_job(q_in, q_out):
    while True:
        occupancy_map, robot_indexes = q_in.get()
        while not q_in.empty():
            occupancy_map, robot_indexes = q_in.get()
        planner = GoalPlanner(occupancy_map)
        goal_point = (0, 0)
        p = planner.closest_frontier_centroid(robot_indexes)
        if p is not None:
            goal_point = p
        q_out.put(goal_point)
        time.sleep(5)

def show_map_job(q_sm, width, height):
    show_map = ShowMap(width, height, True)  # rows, cols, showgui
    while True:
        occupancy_map, laser_model, robot_indexes, goal_point = q_sm.get()
        while not q_sm.empty():
            occupancy_map, laser_model, robot_indexes, goal_point = q_sm.get()
        show_map.updateMap(occupancy_map.grid, laser_model.p_max, robot_indexes[0], robot_indexes[1], goal_point)

def pure_pursuit_job(q_path_out, q_pure_exit):
    #mrds_url = "localhost:50000"
    #controller = FixedController(lookahead=1, mrds_url=mrds_url)
    while True:
        path = q_path_out.get()
        while not q_path_out.empty():
            path = q_path_out.get()
        #logger.info(path)
        #try:
        goFast(path, q_pure_exit)
        #except Exception as e:
        #    logger.warning("PURE PURSUIT CRASHED: " + str(e))
        #logger.info("starting pure pursuit")
        #controller.pure_pursuit(q_pure_exit)
        logger.info("PURE: NEW PATH")

def path_planner_job(q_path_in, q_path_out, q_pure_exit):
    while True:
        occupancy_map, robot_cell, goal_point = q_path_in.get()
        while not q_path_in.empty():
            occupancy_map, robot_cell, goal_point = q_path_in.get()
        path_planner = PathPlanner(occupancy_map)
        path = path_planner.get_path(robot_cell, goal_point)
        logger.info("GOAL POINT: " + str(goal_point))
        x0, y0 = path[0]
        x, y = path[-1]
        logger.info("ROBOT POS: " + str(robot_cell))
        logger.info("FIRST PATH NODE: " + str(occupancy_map.convert_to_grid_indexes(x0, y0)))
        logger.info("LAST PATH NODE: " + str(occupancy_map.convert_to_grid_indexes(x, y)))

        new_path = []
        grid = occupancy_map.grid
        for x, y in path:
            new_node = {}
            new_node['Pose'] = {}
            new_node['Pose']['Position'] = {}
            new_node['Pose']['Position']['X'] = x
            new_node['Pose']['Position']['Y'] = y
            new_path.append(new_node)
        q_path_out.put(new_path)
        
        #q_path_out.put(path)
        if q_pure_exit.empty():
            q_pure_exit.put(1)
        time.sleep(10)

if __name__ == '__main__':
    mrds_url = "localhost:50000"
    scale = 2
    x1 = -60
    y1 = -60
    x2 = 60
    y2 = 60
    width = x2 - x1
    height = y2 - y1
    max_distance = 100

    # TODO: parse arguments and print usage
    if len(argv) == 6:
        x1 = argv[1]
        x2 = argv[2]
        y1 = argv[3]
        y1 = argv[4]
        url = argv[5]
    # else:
    #    print("Usage: mapper ")

        # occupancy_map = Map(width, height, scale)


    controller = FixedController(lookahead=1, mrds_url=mrds_url)

    laser_angles = controller.get_laser_scan_angles()
    laser_model = LaserModel(laser_angles, max_distance)

    occupancy_map = Map(x1, y1, x2, y2, scale, controller.get_pos())

    

    goal_planner = GoalPlanner(occupancy_map)

    pos, rot = controller.get_pos_and_orientation()
    robot_indexes = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)

    q_in = Queue()
    q_out = Queue()

    q_sm = Queue()

    q_path_in = Queue()
    q_path_out = Queue()
    q_pure_exit = Queue()


    p = Process(target=planner_job, args=(q_in, q_out,))
    p.daemon = False
    p.start()
    p2 = Process(target=show_map_job, args=(q_sm, scale * width, scale * height))
    p2.daemon = False
    p2.start()

    p3 = Process(target=path_planner_job, args=(q_path_in, q_path_out, q_pure_exit,))
    p3.daemon = False
    p3.start()

    p4 = Process(target=pure_pursuit_job, args=(q_path_out, q_pure_exit))
    p4.daemon = False
    p4.start()

    goal_point = (0,0)

    while True:
        laser_scan = controller.get_laser_scan()
        pos, rot = controller.get_pos_and_orientation()
        laser_model.apply_model(occupancy_map, pos, rot, laser_scan)
        robot_cell = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)

        if q_sm.empty():
            q_sm.put([occupancy_map, laser_model, robot_cell, goal_point])

        if q_in.empty():
            q_in.put([occupancy_map, robot_cell])
        while not q_out.empty():
            goal_point = q_out.get()

        if q_path_in.empty():
            q_path_in.put([occupancy_map, robot_cell, goal_point])

        time.sleep(0.2)


    p.join()

