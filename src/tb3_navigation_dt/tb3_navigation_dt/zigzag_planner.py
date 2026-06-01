import numpy as np


def generate_zigzag_waypoints(map_data, strip_width: float = 0.5, margin: float = 0.3):
    """
    Generate lawnmower waypoints over the free space in an OccupancyGrid.

    strip_width : metres between parallel sweep lines
    margin      : metres to keep away from the free-space boundary (wall buffer)

    Returns a list of (x, y) world-frame tuples.
    """
    h = map_data.info.height
    w = map_data.info.width
    res = map_data.info.resolution
    ox = map_data.info.origin.position.x
    oy = map_data.info.origin.position.y

    grid = np.array(map_data.data, dtype=np.int8).reshape(h, w)
    free = grid == 0

    free_rows, free_cols = np.where(free)
    if len(free_rows) == 0:
        return []

    margin_cells = max(0, int(margin / res))

    col_min = int(free_cols.min()) + margin_cells
    col_max = int(free_cols.max()) - margin_cells
    row_min = int(free_rows.min()) + margin_cells
    row_max = int(free_rows.max()) - margin_cells

    if col_min >= col_max or row_min >= row_max:
        return []

    x_min = ox + col_min * res
    x_max = ox + col_max * res
    y_min = oy + row_min * res
    y_max = oy + row_max * res

    waypoints = []
    x = x_min
    going_up = True

    while x <= x_max + 1e-6:
        y_start = y_min if going_up else y_max
        y_end = y_max if going_up else y_min
        waypoints.append((x, y_start))
        waypoints.append((x, y_end))
        going_up = not going_up

        next_x = x + strip_width
        # Make sure we hit the far edge even if it doesn't land on a strip boundary
        if x < x_max < next_x:
            x = x_max
        else:
            x = next_x

    return waypoints
