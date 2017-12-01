from logging import getLogger
from time import sleep

from .controller import Controller
from model import Quaternion, Vector, pure_pursuit

logger = getLogger('controller')


class ObstacleController(Controller):
    """
    Class that inherits from Controller and optimizes pure pursuit using obstacle detection.
    """

    def __init__(self, mrds_url, lin_spd, max_lookahead, delta_pos):
        """
        Initializes a new FixedController instance.
        :param mrds_url: url which the MRDS server listens on
        :type mrds_url: str
        :param lin_spd:
        :type lin_spd: float
        :param max_lookahead: maximum number of positions to skip on the path
        :type max_lookahead: int
        :param delta_pos: "close enough" distance. Minimum distance from the target position at which the robot
        considers it reached that position
        :type delta_pos: float

        """
        super(ObstacleController, self).__init__(mrds_url, lin_spd, delta_pos)
        self.__max_lookahead = max_lookahead
        logger.info(
            'Using {} with linear speed={}, max_lookahead={}, delta position={}'.format(self.__class__.__name__, lin_spd,
                                                                                    max_lookahead, delta_pos))

    def pure_pursuit(self, pos_path):
        """
        Implements the pure pursuit algorithm using obstacle detection to aim for the furthest position possible.

        :param pos_path: path loaded into Vector
        :type pos_path: list
        """
        pos_index = -1
        last_pos_index = len(pos_path) - 1
        while pos_index < last_pos_index:
            cur_pos, cur_rot = self.get_pos_and_orientation()

            # aim for the position before the one that would cause a collision
            new_pos_index = self.next_optimized_waypoint(cur_pos, cur_rot, pos_path, pos_index)
            if new_pos_index == pos_index:
                pos_index = new_pos_index + 1
            else:
                pos_index = new_pos_index

            logger.info("Target position path index: {}".format(new_pos_index))

            self.travel(cur_pos, pos_path[pos_index], self._lin_spd,
                        pure_pursuit.get_ang_spd(cur_pos, cur_rot, pos_path[pos_index], self._lin_spd))


        self.stop()

    def next_optimized_waypoint(self, cur_pos, cur_rot, pos_path, cur_pos_index):
        """
        Returns the furthest point from path without an obstacle. Stops at the first position where the laser of
        nearest angle (laser angle ~= aimed position angle) detects an obstacle (laser distance < aimed position
        distance).
        :param cur_pos: current position of the robot
        :type cur_pos: Vector
        :param cur_rot: current orientation of the robot
        :type cur_rot: Quaternion
        :param pos_path: loaded path
        :type pos_path: list
        :param cur_pos_index: index of the current position in the path (-1
        :type cur_pos_index: int

        """
        lasers_angles = self.get_laser_scan_angles()
        lasers = self.get_laser_scan()['Echoes']

        max_lookahead_index = min(cur_pos_index + self.__max_lookahead - 1, len(pos_path) - 1)
        # Go through every position on the path starting at the position next to the current one and stopping at
        # max_lookahead_index positions further
        for i in range(cur_pos_index + 1, max_lookahead_index):
            tar_pos = pos_path[i]
            # convert potential aimed position to RCS
            rcs_tar_pos = pure_pursuit.convert_to_rcs(tar_pos, cur_pos, cur_rot)

            # current robot position in RCS
            rcs_origin = Vector(0, 0, 0)
            # compute angle between current robot position and aimed position
            tar_angle = rcs_origin.get_angle(rcs_tar_pos)

            min_ind = 0
            min_dist = tar_angle - lasers_angles[0]
            # search for the nearest angle in laser_angles
            # could be simplified with a calculation instead of this iteration
            for j in range(1, len(lasers_angles)):
                dist = tar_angle - lasers_angles[j]
                if dist < min_dist:
                    min_dist = dist
                    min_ind = j

            # if the laser hits an obstacle, return the index of the previous position on the path
            if lasers[min_ind] < cur_pos.distance_to(tar_pos):
                return i - 1
        return max_lookahead_index



