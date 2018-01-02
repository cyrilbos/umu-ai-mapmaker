# Temporary testing code
import logging
logging.basicConfig(format="[%(asctime)s %(filename)s:%(lineno)s - %(funcName)20s() ] %(message)s",
                    level=logging.INFO)

logger = logging.getLogger(__name__)

from threading import Thread

from path_planner import PathPlanner


from sys import argv

from mapper import LaserModel
from mapper import Map
from mapper import ShowMap


from controller import FixedController

from planner.planner import Planner
import time

goal_point = None
pos = None
rot = None
robot_indexes = None


def mapping_routine():
    #while True:
    global goal_point, pos, rot, robot_indexes, laser_model, showmap_map
    laser_scan = controller.get_laser_scan()
    #pos, rot = controller.get_pos_and_orientation()
    laser_model.apply_model(occupancy_map, pos, rot, laser_scan)
    #robot_indexes = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)
    showmap_map.updateMap(occupancy_map.grid, laser_model.p_max, robot_indexes[0], robot_indexes[1], goal_point)
    logger.info("updated map")
    time.sleep(0.01)


if __name__ == '__main__':
    mrds_url = "localhost:50000"
    scale = 4
    x1 = -60
    y1 = -60
    x2 = 60
    y2 = 60
    width = x2 - x1
    height = y2 - y1
    #width = 100
    #height = 100
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
    showmap_map = ShowMap(scale * width, scale * height, True)  # rows, cols, showgui

    path_planner = PathPlanner(occupancy_map)
    goal_planner = Planner(occupancy_map)

    pos, rot = controller.get_pos_and_orientation()
    robot_indexes = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)
    goal_point = goal_planner.closest_frontier_centroid(robot_indexes)
    laser_scan = controller.get_laser_scan()
    laser_model.apply_model(occupancy_map, pos, rot, laser_scan)

    #mapping_thread = Thread(target=mapping_routine(), daemon=True)
    #mapping_thread.start()
   # mapping_routine()

    while True:
        pos, rot = controller.get_pos_and_orientation()
        robot_indexes = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)
        mapping_thread = Thread(target=mapping_routine(), daemon=True)
        mapping_thread.start()
        #laser_scan = controller.get_laser_scan()
        #laser_model.apply_model(occupancy_map, pos, rot, laser_scan)

        p = goal_planner.closest_frontier_centroid(robot_indexes)

        if p is not None:
            goal_point = p
            logger.info("new goal point {}".format(p))

        path = path_planner.get_path(robot_indexes, goal_point)
        logger.info("new path {}".format(path))
        controller.set_pos_path(path)

        logger.info("starting pure pursuit")
        controller.pure_pursuit()
        #showmap_map.updateMap(occupancy_map.grid, laser_model.p_max, robot_indexes[0], robot_indexes[1], goal_point)
        time.sleep(0.01)

