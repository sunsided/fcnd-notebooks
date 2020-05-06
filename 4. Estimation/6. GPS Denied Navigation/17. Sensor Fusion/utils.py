import numpy as np

def create_grid(data):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        obstacle = [
            int(np.clip(north - d_north - north_min, 0, north_size-1)),
            int(np.clip(north + d_north - north_min, 0, north_size-1)),
            int(np.clip(east - d_east - east_min, 0, east_size-1)),
            int(np.clip(east + d_east - east_min, 0, east_size-1)),
        ]
        grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)

def inbounds(grid_map, x, y):
    if x >= grid_map.shape[1]:
        return False
    elif y >= grid_map.shape[0]:
        return False
    elif x < 0 or y < 0:
        return False
    return True

def valid_location(grid_map, x, y):
    """Return whether the (x, y) location is inside the map's bounds
    and hasn't hit an obstacle."""
    if grid_map[y, x] == 1:
        return False
    return inbounds(grid_map, x, y)
