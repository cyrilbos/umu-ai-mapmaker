from logging import getLogger

import time

from .controller import Controller
from model import pure_pursuit, Vector

logger = getLogger('controller')


class FixedController(Controller):
    """
    Class that inherits from Controller and implements pure pursuit with a fixed lookahead.
    """

    def __init__(self, lookahead=5, *args, **kwargs):
        """
        Initializes a new FixedController instance.
        :param mrds_url: url which the MRDS server listens on
        :type mrds_url: str
        :param lin_spd:
        :type lin_spd: float
        :param lookahead: fixed number of positions to skip on the path
        :type lookahead: int
        :param delta_pos: "close enough" distance. Minimum distance from the target position at which the robot
        considers it reached that position
        :type delta_pos: float

        """
        super(FixedController, self).__init__(*args, **kwargs)
        self.__lookahead = lookahead


    def pure_pursuit(self):
        """
        Implements the pure pursuit algorithm with a fixed lookahead. The robot aims for "self.__lookahead"
        positions ahead on the given path.

        :param pos_path: list of Vector
        :type pos_path: list
        """
        # Travel through the path skipping "lookahead" positions every time
        for i in range(0, len(self._pos_path), self.__lookahead):
            cur_pos, cur_rot = self.get_pos_and_orientation()
            tar_pos = Vector(self._pos_path[i][0], self._pos_path[i][1], cur_pos.z)
            logger.info("Travelling to {}".format(tar_pos))
            self.travel(cur_pos, tar_pos, self._lin_spd,
                        pure_pursuit.get_ang_spd(cur_pos, cur_rot, tar_pos, self._lin_spd))
        self.stop()

