import matplotlib.pyplot as plt
import numpy as np

def calculate_segment_distances(waypoints):
    segment_distances = []
    for i in range(len(waypoints) - 1):
        x0, y0 = waypoints[i]
        x1, y1 = waypoints[i + 1]
        distance = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        segment_distances.append(distance)
    return segment_distances

def interpolate_points(waypoints, total_points):
    num_segments = len(waypoints) - 1
    segment_distances = calculate_segment_distances(waypoints)
    total_distance = sum(segment_distances)
    interpolated_points = []
    for i in range(num_segments):
        x0, y0 = waypoints[i]
        x1, y1 = waypoints[i + 1]
        segment_points = int(total_points * (segment_distances[i] / total_distance))
        for j in range(segment_points + 1):
            alpha = j / segment_points
            x = x0 + alpha * (x1 - x0)
            y = y0 + alpha * (y1 - y0)
            interpolated_points.append([x, y])  # Changed to list format
    return interpolated_points

def plot_points(points, waypoints, waypoint_color='red', generated_point_color='blue'):
    # Plot waypoints
    plt.plot([point[0] for point in waypoints], [point[1] for point in waypoints], 'o-', color=waypoint_color, label='Waypoints')
    # Plot generated points
    plt.plot([point[0] for point in points], [point[1] for point in points], 'o-', color=generated_point_color, label='Generated Points')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Waypoints and Generated Points')
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    # Example usage:
    waypoints = [[1,1], [1,30], [5,30], [5,1], [10,1], [10,30], [15,30], [15,1], [20,1], [20,30], [25,30], [25,1], [30,1], [30,30], [35,30], [35,1], [40,1], [40,30], [45,30], [45,1], [50,1], [50,30], [55,30], [55,1], [60,1], [60,30], [65,30], [65,1], [70,1], [70,30], [75,30], [75,1], [80,1], [80,30], [85,30], [85,1], [90,1], [90,30], [95,30], [95,1], [100,1], [100,30]] # Changed to 2-dimensional array format
    total_points_desired = 5000  # Specify the total number of points desired
    generated_points = interpolate_points(waypoints, total_points_desired)
    print("Generated points between waypoints:")
    for point in generated_points:
        print(point)

    # Plot waypoints and generated points
    plot_points(generated_points, waypoints)
