import time

from mapper import ShowMap


def show_map_job(q_sm, width, height, q_showmap_path):
    show_map = ShowMap(width, height, True)  # rows, cols, showgui
    # shows map with obstacles expanded by 2 cells
    # show_expanded_map = ShowMap(width, height, True, name="Expanded map")
    goal_point = (0, 0)
    path = None
    while True:
        occupancy_map, laser_model, robot_cell = q_sm.get()
        while not q_sm.empty():
            occupancy_map, laser_model, robot_cell = q_sm.get()
        if not q_showmap_path.empty():
            while not q_showmap_path.empty():
                path, goal_point = q_showmap_path.get()
        show_map.updateMap(occupancy_map.grid, laser_model.p_max, robot_cell[0], robot_cell[1], goal_point, path)
        # uncomment this to show the expanded map
        # show_expanded_map.updateMap(occupancy_map.obstacle_expanded_map().obstacle_expanded_map().grid, laser_model.p_max, robot_cell[0], robot_cell[1], goal_point, path)
        time.sleep(0.5)
