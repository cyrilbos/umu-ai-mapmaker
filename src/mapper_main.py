# Temporary testing code

import logging


logging.basicConfig(format="[%(asctime)s %(filename)s:%(lineno)s - %(funcName)20s() ] %(message)s",
                    level=logging.INFO)
from sys import argv

from mapper import LaserModel
from mapper import Map
from mapper import ShowMap


from controller import Controller


from planner.planner import Planner
import time


if __name__ == '__main__':
    mrds_url = "localhost:50000"
    scale = 4
    x1 = -15
    y1 = -15
    x2 = 15
    y2 = 15
    width = x2 - x1
    height = y2 - y1
    #width = 100
    #height = 100
    max_distance = 30

    # TODO: parse arguments and print usage
    if len(argv) == 4:
        width = argv[1]
        height = argv[2]
    # else:
    #    print("Usage: mapper ")

        # occupancy_map = Map(width, height, scale)
    occupancy_map = Map(x1, y1, x2, y2, scale)
    showmap_map = ShowMap(scale * width, scale * height, True)  # rows, cols, showgui

    controller = Controller(mrds_url)
    laser_angles = controller.get_laser_scan_angles()
    laser_model = LaserModel(laser_angles, max_distance)

    while True:
        laser_scan = controller.get_laser_scan()
        pos, rot = controller.get_pos_and_orientation()
        laser_model.apply_model(occupancy_map, pos, rot, laser_scan)
        robot_indexes = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)
        planner = Planner(occupancy_map)
        goal_point = (0, 0)
        p = planner.closest_frontier_centroid(robot_indexes)

        if p is not None:
            goal_point = p

        # TODO: p max getter
        showmap_map.updateMap(occupancy_map.grid(), laser_model._p_max, robot_indexes[0], robot_indexes[1])
        time.sleep(0.01)
