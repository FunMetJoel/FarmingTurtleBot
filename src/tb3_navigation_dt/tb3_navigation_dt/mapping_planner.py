import numpy as np
import math
import matplotlib.pyplot as plt

def generate_mapping_waypoints(occupancy_map_msg, humidity_map_msg):
    """
    Generate waypoints to visit each cell in the humidity map that has nothing in the occupancy map.

    Returns a list of (x, y) world-frame tuples.
    """

    cell_size = humidity_map_msg.info.resolution
    low_res_occupancy_grid = _generate_low_res_occupancy_grid(occupancy_map_msg, cell_size=cell_size)
    reachable_grid = _generate_reachability_grid(occupancy_map_msg)

    h = low_res_occupancy_grid.shape[0]
    w = low_res_occupancy_grid.shape[1]
    waypoints = []

    orgX = math.floor(occupancy_map_msg.info.origin.position.x * (1/cell_size)) * cell_size
    orgY = math.floor(occupancy_map_msg.info.origin.position.y * (1/cell_size)) * cell_size
    for i in range(h):
        for j in range(w):
            if not low_res_occupancy_grid[i, j]:  # If the cell is not occupied
                
                x = orgX + j * cell_size
                y = orgY + i * cell_size

                # check of this coordinate x y is reachable using the reachable_grid
                grid_x = round((x - occupancy_map_msg.info.origin.position.x) / occupancy_map_msg.info.resolution)
                grid_y = round((y - occupancy_map_msg.info.origin.position.y) / occupancy_map_msg.info.resolution)
                if not (0 <= grid_x < reachable_grid.shape[0] and 0 <= grid_y < reachable_grid.shape[1]):
                    continue
                if not reachable_grid[grid_x, grid_y]:
                    continue

                waypoints.append((x, y))

    return waypoints, _toOccupancyGridData(low_res_occupancy_grid, 0, 1)


def _generate_low_res_occupancy_grid(occupancy_map_msg, cell_size):
    """
    Generate a low-resolution occupancy grid where a cell is considered occupied if any point in the corresponding area of the original occupancy grid is occupied.

    The cell at (0, 0) in the low-res grid corresponds to the area from (-cell_size/2, -cell_size/2) to (cell_size/2, cell_size/2) in the original grid.

    Returns a 2D numpy array of booleans where True indicates occupied and False indicates free.
    """

    h = occupancy_map_msg.info.height
    w = occupancy_map_msg.info.width
    res = occupancy_map_msg.info.resolution # m/cell
    ox = occupancy_map_msg.info.origin.position.x
    oy = occupancy_map_msg.info.origin.position.y

    grid = np.array(occupancy_map_msg.data, dtype=np.int8).reshape(h, w)

    low_res_h = int(np.floor((h * res) / cell_size))
    low_res_w = int(np.floor((w * res) / cell_size))
    low_res_grid = np.zeros((low_res_h, low_res_w), dtype=bool)

    for i in range(low_res_h):
        for j in range(low_res_w):
            x_min = ox + j * cell_size - cell_size / 2
            x_max = ox + j * cell_size + cell_size / 2
            y_min = oy + i * cell_size - cell_size / 2
            y_max = oy + i * cell_size + cell_size / 2

            col_min = max(0, int(np.floor((x_min - ox) / res)))
            col_max = min(w - 1, int(np.ceil((x_max - ox) / res)))
            row_min = max(0, int(np.floor((y_min - oy) / res)))
            row_max = min(h - 1, int(np.ceil((y_max - oy) / res)))

            if np.any(grid[row_min:row_max+1, col_min:col_max+1] > 0):
                low_res_grid[i, j] = True

    return low_res_grid

def _generate_reachability_grid(occupancy_map_msg):
    """
    Uses the occupancy grid to determine if a certain part of the grid would be reachable by the robot.
    Any points that are surrounded by walls are not reachable.
    Assume the robot starts at (0, 0).

    Returns a numpy array the same shape as the occupancy map, where reachable free cells are True
    and all other cells are False.
    """

    h = occupancy_map_msg.info.height
    w = occupancy_map_msg.info.width
    res = occupancy_map_msg.info.resolution # m/cell
    ox = occupancy_map_msg.info.origin.position.x
    oy = occupancy_map_msg.info.origin.position.y

    grid = np.array(occupancy_map_msg.data, dtype=np.int8).reshape(h, w)
    occupied = (grid > 0) | (grid == -1)

    start_col = int(np.floor((0.0 - ox) / res))
    start_row = int(np.floor((0.0 - oy) / res))

    reachable = np.zeros((h, w), dtype=bool)

    if not (0 <= start_row < h and 0 <= start_col < w):
        return reachable

    if occupied[start_row, start_col]:
        return reachable

    stack = [(start_row, start_col)]
    reachable[start_row, start_col] = True

    while stack:
        row, col = stack.pop()

        for drow, dcol in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nrow = row + drow
            ncol = col + dcol

            if 0 <= nrow < h and 0 <= ncol < w:
                if not reachable[nrow, ncol] and not occupied[nrow, ncol]:
                    reachable[nrow, ncol] = True
                    stack.append((nrow, ncol))
    
    return reachable


def _toOccupancyGridData(array, min:float, max:float):
        occupancyGridData = np.empty((array.shape[0], array.shape[1]), dtype=np.int8) # Array with values between -1 and 100
       
        scale:float = max - min
        occupancyGridData = ((array - min) * (100.0 / scale)).clip(0, 100).round()
    
        occupancyGridData[np.isnan(array)] = -1

        return occupancyGridData.astype(int)