# Temporary testing code

import logging
import time

logging.basicConfig(format="[%(asctime)s %(filename)s:%(lineno)s - %(funcName)20s() ] %(message)s",
                    level=logging.INFO)
from sys import argv
from multiprocessing import Process, Queue

from mapper import LaserModel
from mapper import Map
from mapper import ShowMap
from logging import getLogger


from controller import Controller

from planner.planner import Planner


logger = getLogger(__name__)

def planner_job(q_in, q_out):
    while True:
        occupancy_map, robot_indexes = q_in.get()
        while not q_in.empty():
            occupancy_map, robot_indexes = q_in.get()
        planner = Planner(occupancy_map)
        goal_point = (0, 0)
        p = planner.closest_frontier_centroid(robot_indexes)
        if p is not None:
            goal_point = p
        q_out.put(goal_point)
        time.sleep(5)

def show_map_job(q_sm, width, height):
    showmap_map = ShowMap(width, height, True)  # rows, cols, showgui
    while True:
        occupancy_map, laser_model, robot_indexes, goal_point = q_sm.get()
        while not q_sm.empty():
            occupancy_map, laser_model, robot_indexes, goal_point = q_sm.get()
        showmap_map.updateMap(occupancy_map.grid(), laser_model._p_max, robot_indexes[0], robot_indexes[1], goal_point)


if __name__ == '__main__':
    mrds_url = "localhost:50000"
    scale = 2
    x1 = -50
    y1 = -50
    x2 = 50
    y2 = 50
    width = x2 - x1
    height = y2 - y1
    #width = 100
    #height = 100
    max_distance = 100

    # TODO: parse arguments and print usage
    if len(argv) == 4:
        width = argv[1]
        height = argv[2]
    # else:
    #    print("Usage: mapper ")

        # occupancy_map = Map(width, height, scale)
    occupancy_map = Map(x1, y1, x2, y2, scale)
    

    controller = Controller(mrds_url)
    laser_angles = controller.get_laser_scan_angles()
    laser_model = LaserModel(laser_angles, max_distance)

    q_in = Queue()
    q_out = Queue()

    q_sm = Queue()

    p = Process(target=planner_job, args=(q_in, q_out,))
    p.start()
    p2 = Process(target=show_map_job, args=(q_sm, scale * width, scale * height))
    p2.start()

    goal_point = (0, 0)

    while True:
        laser_scan = controller.get_laser_scan()
        pos, rot = controller.get_pos_and_orientation()
        laser_model.apply_model(occupancy_map, pos, rot, laser_scan)
        robot_indexes = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)

        if q_sm.empty():
            q_sm.put([occupancy_map, laser_model, robot_indexes, goal_point])

        if (q_in.empty()):
            q_in.put([occupancy_map, robot_indexes])
        while not q_out.empty():
            goal_point = q_out.get()

        # TODO: p max getter

    p.join()
