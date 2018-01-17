import time
import logging
from sys import argv
from multiprocessing import Queue, Process

from mapper import LaserModel, Map
from controller import Controller
from mapper import show_map_job
from planner import planning_job

logging.basicConfig(format="[%(asctime)s %(filename)s:%(lineno)s - %(funcName)20s() ] %(message)s",
                    level=logging.INFO)
logger = logging.getLogger(__name__)

if __name__ == '__main__':
    scale = 2  # resolution, i.e number of cells in the cspace grid for each meter
    laser_max_distance = 40

    if len(argv) == 6:
        mrds_url = argv[1]
        x1 = int(argv[2])
        y1 = int(argv[3])
        x2 = int(argv[4])
        y2 = int(argv[5])
    else:
        print("Usage: python3 mapper_main.py url x1 y1 x2 y2")
        exit()

    width = x2 - x1
    height = y2 - y1
    sound = False  # can be set to True to play a beep when the reactive stop activates

    controller = Controller(mrds_url=mrds_url)

    laser_angles = controller.get_laser_scan_angles()
    laser_model = LaserModel(laser_angles, laser_max_distance)

    occupancy_map = Map(x1, y1, x2, y2, scale)

    pose = controller.getPose()['Pose']
    controller.post_speed(2, 0.6)
    timer = 1
    # Initial scan to choose a better first frontier
    while timer > 0:
        laser_model.apply_model(occupancy_map, pose['Position'], pose['Orientation'], controller.get_laser_scan())
        time.sleep(0.1)
        timer -= 0.1
    controller.stop()

    q_sm = Queue()
    q_path_in = Queue()
    q_showmap_path = Queue()

    show_map_process = Process(target=show_map_job, args=(q_sm, scale * width, scale * height, q_showmap_path,))
    show_map_process.daemon = False
    show_map_process.start()

    planning_process = Process(target=planning_job,
                               args=(controller, q_path_in, q_showmap_path, scale, width, height, sound))
    planning_process.daemon = True
    planning_process.start()

    while planning_process.is_alive():
        laser_scan = controller.get_laser_scan()
        pose = controller.getPose()['Pose']
        pos = pose['Position']
        laser_model.apply_model(occupancy_map, pos, pose['Orientation'], laser_scan)
        robot_cell = occupancy_map.convert_to_grid_indexes(pos['X'], pos['Y'])

        if q_sm.empty():
            q_sm.put([occupancy_map, laser_model, robot_cell])
        while not q_path_in.empty():
            q_path_in.get()
        q_path_in.put([occupancy_map, robot_cell])

        time.sleep(0.05)

    show_map_process.terminate()
