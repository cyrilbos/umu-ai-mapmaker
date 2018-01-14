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

logging.basicConfig(format="[%(asctime)s %(filename)s:%(lineno)s - %(funcName)20s() ] %(message)s",
                    level=logging.INFO)
logger = logging.getLogger(__name__)

##############################################################
# Didn't write this A* implementation, so I don't think we can use it,
# but it's useful for testing.
# (code.activestate.com/recipes/578919-python-a-pathfinding-with-binary-heap/)
def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))
    
    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))
                
    return False
#################################################################

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
        #expanded_map = occupancy_map.navigation_map()
        show_map.updateMap(occupancy_map.grid, laser_model.p_max, robot_cell[0], robot_cell[1], goal_point, path)

def path_planner_job(q_path_in, q_showmap_path, mrds_url, starting_pos):
    while True:
        occupancy_map, robot_cell = q_path_in.get()
        while not q_path_in.empty():
            occupancy_map, robot_cell = q_path_in.get()

        logger.info("Finding goal point")
        planner = GoalPlanner(occupancy_map.navigation_map())
        goal_point = starting_pos
        p = planner.closest_frontier_centroid(robot_cell)
        if p is not None:
            goal_point = p
        
        q_showmap_path.put([None, goal_point])

        if goal_point:
            #logger.info("goal point: " + str(goal_point))
            #logger.info("path: " + str(astar(occupancy_map.navigation_map().grid, robot_cell, goal_point)))
            logger.info("Calculating path")
            #path_planner = PathPlanner(occupancy_map.navigation_map())#PathPlanner(Map.expanded_obstacles_map(occupancy_map))
            #path = path_planner.get_path(robot_cell, goal_point)
            path = astar(occupancy_map.navigation_map().grid, robot_cell, goal_point)
            if not path:
                continue
            if len(path) <= 1:
                continue

            # For compatibility with the pure pursuit implementation
            new_path = []
            for xg, yg in path[1:]:
                #x, y = occupancy_map.convert_to_real_position(xg, yg)
                x, y = occupancy_map.center_of_cell(xg, yg)
                new_node = {}
                new_node['Pose'] = {}
                new_node['Pose']['Position'] = {}
                new_node['Pose']['Position']['X'] = x
                new_node['Pose']['Position']['Y'] = y
                new_path.append(new_node)
            new_path.reverse()

            # Send the path to the gui
            #coord_path = []
            #for x, y in path:
            #    coord_path.append(occupancy_map.convert_to_grid_indexes(x, y))
            #q_showmap_path.put([coord_path, goal_point])

            q_showmap_path.put([path, goal_point])

            logger.info("Following path using pp")
            goFast(new_path, mrds_url)

if __name__ == '__main__':
    scale = 2
    laser_max_distance = 40

    if len(argv) == 6:
        mrds_url = argv[1]
        x1 = int(argv[2])
        y1 = int(argv[3])
        x2 = int(argv[4])
        y2 = int(argv[5])
        width = x2 - x1
        height = y2 - y1
    else:
        #print("Usage: python3 mapper_main.py url x1 y1 x2 y2")
        #exit()
        mrds_url = "localhost:50000"
        x1 = -20
        y1 = -20
        x2 = 40
        y2 = 40
        width = x2 - x1
        height = y2 - y1

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
    p2 = Process(target=path_planner_job, args=(q_path_in, q_showmap_path,mrds_url, starting_pos,))
    p2.daemon = False
    p2.start()

    goal_point = (0, 0)
    while True:#TODO: while xhere are unknown cells on the map/no goals?
        laser_scan = controller.get_laser_scan()
        pos, rot = controller.get_pos_and_orientation()
        laser_model.apply_model(occupancy_map, pos, rot, laser_scan)
        robot_cell = occupancy_map.convert_to_grid_indexes(pos.x, pos.y)

        if q_sm.empty():
            q_sm.put([occupancy_map, laser_model, robot_cell])
        while not q_path_in.empty():
            q_path_in.get()
        q_path_in.put([occupancy_map, robot_cell])

        time.sleep(0.075)

    p.join()

