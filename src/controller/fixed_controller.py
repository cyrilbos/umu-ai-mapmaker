from logging import getLogger

from .controller import Controller
from model import pure_pursuit

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
        path = self.get_pos_path()
        while True:
            new_path = self.get_pos_path()
            if new_path != path:
                path = self._pos_path
            if path:
                # Travel through the path skipping "lookahead" positions every time
                for i in range(0, len(path), self.__lookahead):
                    new_path = self.get_pos_path()
                    if new_path != path:
                        self.stop()
                        path = self._pos_path
                        break
                    cur_pos, cur_rot = self.get_pos_and_orientation()
                    self.travel(cur_pos, path[i], self._lin_spd,
                                pure_pursuit.get_ang_spd(cur_pos, cur_rot, path[i], self._lin_spd))

