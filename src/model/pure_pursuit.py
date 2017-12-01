from math import atan2, cos, sin

from .vector import Vector
from .quaternion import Quaternion


def convert_to_rcs(tar_pos, cur_pos, cur_rot):
    """
    Computes the targeted position from the world coordinate system to the robot coordinate system
    :param tar_pos: targeted position to travel to
    :type tar_pos: Vector
    :param cur_pos: current position of the robot
    :type cur_pos: Vector
    :param cur_rot: current rotation of the robot
    :type cur_rot: Quaternion
    """
    q = Quaternion(cur_rot.w, Vector(0, 0, cur_rot.z)).heading()
    angle = atan2(q.y, q.x)
    rcs_pos = Vector(0, 0, tar_pos.z)

    rcs_pos.x = (tar_pos.x - cur_pos.x) * cos(angle) + (tar_pos.y - cur_pos.y) * sin(angle)
    rcs_pos.y = -(tar_pos.x - cur_pos.x) * sin(angle) + (tar_pos.y - cur_pos.y) * cos(angle)

    return rcs_pos

def get_ang_spd(cur_pos, cur_rot, tar_pos, lin_spd):
    """
    Computes the angular speed using pure pursuit formulas
    :param cur_pos: current position of the robot
    :type cur_pos: Vector
    :param cur_rot: current rotation of the robot
    :type cur_rot: Quaternion
    :param tar_pos: targeted position to travel to
    :type tar_pos: Vector
    :param lin_spd: linear speed of the robot
    :type lin_spd: float
    """
    rcs_tar_pos = convert_to_rcs(tar_pos, cur_pos, cur_rot)

    ang_spd = lin_spd / ((pow(rcs_tar_pos.x, 2) + pow(rcs_tar_pos.y, 2)) / (2 * rcs_tar_pos.y))

    return ang_spd

