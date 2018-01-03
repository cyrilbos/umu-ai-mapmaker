import time

from mapper import Map
from model import Vector
from path_planner.path_planner import PathPlanner

import logging
logging.basicConfig(format="[%(asctime)s %(filename)s:%(lineno)s - %(funcName)20s() ] %(message)s",
                    level=logging.DEBUG)

if __name__ == '__main__':
    cspace_map = Map(-10, -10, 10, 10, 4)
    planner = PathPlanner(cspace_map)
    time.sleep(5)
    path = planner.get_path((0,0), (8,9))
    print(path)

    coord_path = planner._graph.to_coordinates_path(path)
    print(coord_path)
