from planner.planner import Planner
from mapper.map import Map

# Temporary testing code
if __name__ == '__main__':
    cspace_map = Map(-10, -10, 10, 10, 4)
    planner = Planner()
    # print(planner.find_open(cspace_map)) # seems to work correctly
    # print(planner.has_neighbor_in((3, 3), [(4, 4)])) # works
    # print(planner.adjacent((3, 3))) # works