# Temporary testing code

import logging

from controller import Controller

logging.basicConfig(format="[%(filename)s:%(lineno)s - %(funcName)20s() ] %(message)s",
                    level=logging.INFO)
from sys import argv

from mapper.laser_model import LaserModel
from mapper.mapper import Map
from mapper.show_map import ShowMap
from mapper.util import getLaserAngles, getLaser, getPose
import time

if __name__ == '__main__':
    mrds_url = "localhost:50000"
    scale = 4
    x1 = -30
    y1 = -30
    x2 = 30
    y2 = 30
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
    laser_angles = getLaserAngles()
    laser_model = LaserModel(laser_angles, max_distance)

    controller = Controller(mrds_url)
    while True:
        laser_scan = getLaser()
        pos, rot = controller.get_pos_and_orientation()
        laser_model.apply_model(occupancy_map, pos, rot, laser_scan)
        robot_indexes = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)
        #TODO: p max getter
        showmap_map.updateMap(occupancy_map.grid(), laser_model._p_max, robot_indexes[0], robot_indexes[1])
        time.sleep(0.01)
