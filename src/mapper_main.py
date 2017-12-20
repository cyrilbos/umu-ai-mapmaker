# Temporary testing code
import logging

from mapper.LaserModel import LaserModel
from mapper.mapper import Map
from mapper.show_map import ShowMap
from mapper.util import getLaserAngles, getLaser, getPose

if __name__ == '__main__':
    scale = 4
    width = 20
    height = 20
    occupancy_map = Map(width, height, scale)
    showmap_map = ShowMap(scale * width, scale * height, True) # rows, cols, showgui
    logging.basicConfig(level=logging.DEBUG)
    laser_angles = getLaserAngles()
    laser_model = LaserModel(laser_angles)
    while True:
        laser_scan = getLaser()
        pose = getPose()
        laser_model.apply_model(occupancy_map, pose, laser_scan)
        showmap_map.updateMap(occupancy_map._map, 15,
            pose['Pose']['Position']['X'] * 4, pose['Pose']['Position']['Y'] * 4)
        #time.sleep(1)