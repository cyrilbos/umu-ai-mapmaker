import logging
import argparse
import time

from model import Vector, Quaternion
from controller import FixedController, ObstacleController, PathLoader

# url which the MRDS server listens on
mrds_url = 'localhost:50000'

# filename of the path to load. Can be set by appending option --path=filename to this script
path_filepath = 'paths/Path-around-table-and-back.json'

# can be set to True by appending argument --obstacle to this script
# if set to True, instead of a fixed lookahead it will try to optimize as much as possible by detecting obstacles
# if set to False, the controller will skip positions with a fixed lookahead independent of the obstacle detection
obstacle_detection = False

# Optimized parameters for each path
# the lists respect the format [lin_spd, lookahead, delta_pos] when using fixed lookahead
# otherwise for the obstacle detection the format is [linear_speed, max_lookahead, delta_pos]
# lin_spd (linear speed) and delta_pos parameters are described in Controller.__init__()
# lookahead is described in FixedController.__init__() and used in FixedController.pure_pursuit()
# max_lookahead is described in ObstacleController.__init__() and used in ObstacleController.next_optimized_waypoint()
# if using another filename, will use the default values of Controller.__init__
PARAMETERS = {
    'fixed': {
        'Path-around-table-and-back': [1.5, 10, 0.75],
        'Path-around-table': [1, 5, 0.75],
        'Path-to-bed': [1, 5, 0.75],
        'Path-from-bed': [1, 5, 1],
    },
    'obstacle': {
        'Path-around-table-and-back': [1.5, 30, 0.75],
        'Path-around-table': [1, 60, 1],
        'Path-to-bed': [1, 8, 0.75],
        'Path-from-bed': [1, 10, 1],
        'exam2017': [1, 50, 0.75]
    }
}

# default values for Fixed
FIXED_DEFAULT_LIN_SPD = 0.5
FIXED_DEFAULT_LOOKAHEAD = 5
FIXED_DEFAULT_DELTA_POS = 0.75

# default values for ObstacleController
OBSTACLE_DEFAULT_LIN_SPD = 0.75
OBSTACLE_DEFAULT_MAX_LOOKAHEAD = 5
OBSTACLE_DEFAULT_DELTA_POS = 0.75

#Used for logging level option
LOG_LEVEL_STRINGS = ['CRITICAL', 'ERROR', 'WARNING', 'INFO', 'DEBUG']
logging.basicConfig(level=logging.INFO)

if __name__ == '__main__':
    # Parsing arguments for script options
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str, help='filename of the path to load')
    parser.add_argument('--obstacle', action='store_true', default=False,
                        help='use obstacle detection to optimize the path')
    parser.add_argument('--level', type=str, help='Python logger level (ERROR, INFO, DEBUG). Defaults to info. \
                                                  Setting to debug will provide more information such as current position \
                                                   of the robot (among others). \
                                                  Setting to error will provide less information (only exception catching). ')

    args = parser.parse_args()

    logger = logging.getLogger(__name__)
    # Setting up depending on options values
    if args.obstacle:
        obstacle_detection = True
    if args.path:
        path_filepath = args.path
    if args.level:
        logger.setLevel(args.level)
        logging.getLogger('controller').setLevel(LOG_LEVEL_STRINGS.index(args.level))

    # if not placed in same folder, parses the filename of the filepath of the path
    # used to get the optimized parameters for each path
    if '/' in path_filepath:
        path_name = path_filepath[path_filepath.rindex('/') + 1:path_filepath.rindex('.')]
    elif '\\' in path_filepath:
        path_name = path_filepath[path_filepath.rindex('\\') + 1:path_filepath.rindex('.')]
    else:
        path_name = path_filepath
    logger.debug('Filename of path: ' + path_name)

    # Load the path
    try:
        logger.info('Loading path: {}'.format(path_filepath))
        path_loader = PathLoader(path_filepath)
    except Exception as ex:
        logger.error('Failed to load path {}:\n {}'.format(path_filepath, ex))
        exit()

    pos_path = path_loader.positionPath()

    logger.info('Sending commands to MRDS server listening at {}'.format(mrds_url))

    # Instantiate the chosen Controller with optimized parameters, or default ones
    if obstacle_detection:
        if path_name in PARAMETERS['obstacle']:
            controller = ObstacleController(mrds_url, PARAMETERS['obstacle'][path_name][0],
                                            PARAMETERS['obstacle'][path_name][1], PARAMETERS['obstacle'][path_name][2])
        else:
            controller = ObstacleController(mrds_url, OBSTACLE_DEFAULT_LIN_SPD, OBSTACLE_DEFAULT_MAX_LOOKAHEAD,
                                            OBSTACLE_DEFAULT_DELTA_POS)
        logger.info('Starting obstacle optimized pure pursuit')
    else:
        if path_name in PARAMETERS['fixed']:
            controller = FixedController(mrds_url, PARAMETERS['fixed'][path_name][0], PARAMETERS['fixed'][path_name][1],
                                         PARAMETERS['fixed'][path_name][2])
        else:
            controller = FixedController(mrds_url, FIXED_DEFAULT_LIN_SPD, FIXED_DEFAULT_LOOKAHEAD,
                                         FIXED_DEFAULT_DELTA_POS)
        logger.info('Starting fixed lookahead pure pursuit')

    if path_name == "Path-from-bed":
        controller.u_turn()

    # Start stopwatch and start the controller logic sending instructions to the robot
    begin_time = time.time()
    controller.pure_pursuit(pos_path)
    end_time = time.time()

    logger.info('Path done in {}'.format(end_time - begin_time))