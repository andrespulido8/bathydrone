import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, voronoi_plot_2d

# Define the vertices of the polygon and obstacles
polygon_vertices = [(0, 0), (0, 10), (10, 10), (10, 0)]
obstacles = [(3, 3), (3, 7), (7, 7), (7, 3)]

# Define the grid size
grid_size = 0.5

# Define the grid boundaries based on the polygon vertices and obstacles
min_x, min_y = np.min(np.concatenate((polygon_vertices, obstacles), axis=0), axis=0)
max_x, max_y = np.max(np.concatenate((polygon_vertices, obstacles), axis=0), axis=0)
grid_x, grid_y = np.meshgrid(np.arange(min_x, max_x + grid_size, grid_size),
                             np.arange(min_y, max_y + grid_size, grid_size))
grid_shape = grid_x.shape

# Define a function to check if a point is inside the polygon or any obstacle
def is_inside_polygon_or_obstacle(point, polygon_vertices, obstacles):
    if is_inside_polygon(point, polygon_vertices):
        return True
    for obstacle in obstacles:
        if is_inside_polygon(point, obstacle):
            return True
    return False

# Define a function to check if a point is inside a polygon
def is_inside_polygon(point, vertices):
    n = len(vertices)
    inside = False
    p1x, p1y = vertices[0]
    for i in range(n + 1):
        p2x, p2y = vertices[i % n]
        if point[1] > min(p1y, p2y):
            if point[1] <= max(p1y, p2y):
                if point[0] <= max(p1x, p2x):
                    if p1y != p2y:
                        x_inters = (point[1] - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or point[0] <= x_inters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

# Define a function to get the next unvisited grid point
def get_next_point(current_point, visited):
    x, y = current_point
    dx, dy = np.meshgrid([-1, 0, 1], [-1, 0, 1])
    dx, dy = dx.flatten(), dy.flatten()
    distances = np.sqrt((dx**2) + (dy**2))
    sort_indices = np.argsort(distances)
    for i in sort_indices:
        new_x, new_y = x + dx[i], y + dy[i]
        if (0 <= new_x < grid_shape[1]) and (0 <= new_y < grid_shape[0]) and \
           ((new_x, new_y) not in visited) and not is_inside_polygon_or_obstacle((grid_x[new_y, new_x], grid_y[new_y, new_x]), polygon_vertices, obstacles):
            return (new_x, new_y)
    return None

# Generate the coverage path
start_point = (0, 0)
visited = [start_point]
path = [start_point]
while True:
    next_point = get_next_point(path[-1], visited)
    if next_point is:
