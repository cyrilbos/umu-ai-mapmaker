from planner.planner import Planner
from mapper.mapper import Map



# Temporary testing code
if __name__ == '__main__':
    cspace_map = Map(-10, -10, 10, 10, 4)
    planner = Planner()
    # print(planner.find_open(cspace_map)) # seems to work correctly