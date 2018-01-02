from sys import argv
import time
from multiprocessing import Queue, Process


from threading import Thread

from path_planner import PathPlanner

from mapper import LaserModel, Map, ShowMap

from controller import FixedController

from goal_planner import GoalPlanner

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

    path_planner = PathPlanner(occupancy_map)

    goal_planner = GoalPlanner(occupancy_map)

    pos, rot = controller.get_pos_and_orientation()
    robot_indexes = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)

    q_in = Queue()
    q_out = Queue()

    q_sm = Queue()

    p = Process(target=planner_job, args=(q_in, q_out,))
    p.start()
    p2 = Process(target=show_map_job, args=(q_sm, scale * width, scale * height))
    p2.start()

    goal_point = (0,0)

    while True:
        laser_scan = controller.get_laser_scan()
        pos, rot = controller.get_pos_and_orientation()
        laser_model.apply_model(occupancy_map, pos, rot, laser_scan)
        robot_cell = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)

        if q_sm.empty():
            q_sm.put([occupancy_map, laser_model, robot_cell, goal_point])

        if (q_in.empty()):
            q_in.put([occupancy_map, robot_cell])
        while not q_out.empty():
            goal_point = q_out.get()

        path = path_planner.get_path(robot_cell, goal_point)
        controller.set_pos_path(path)
        logger.info("new path {}".format(path))

        logger.info("starting pure pursuit")
        controller.pure_pursuit()
    p.join()

