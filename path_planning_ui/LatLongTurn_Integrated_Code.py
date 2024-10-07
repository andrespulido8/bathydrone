import numpy as np
import math
import matplotlib.pyplot as plt

from mission_planner_code_python import plan_mission

def bi_linear_interpolation(points_reference, points_targeted):
    """ Bi-linear interpolation for latitude and longitude values
        This function takes in 4 reference points and targeted points and returns the interpolated latitude and longitude values
        for the targeted points. The bilinear interpolation is done by first calculating the slope and bias for latitude and longitude
        values and then using these values to interpolate the latitude and longitude values for the targeted points.
        
        Args:
            points_reference: 4 reference points in the form of [x, y, latitude, longitude]
            points_targeted: targeted points in the form of [x, y]
        
        Returns:
            lat_long_interpolation: Interpolated latitude and longitude values for the targeted points"""

    xy = points_reference[:, :2]
    ll = points_reference[:, 2:]

    print("points_reference: ", points_reference)
    lat_slope = (ll[3, 0] - ll[2, 0]) / (xy[3, 1] - xy[2, 1])  # Latitude slope is between top and bottom points
    long_slope = (ll[1, 1] - ll[0, 1]) / (xy[1, 0] - xy[0, 0])  # Longitude slope is between right and left points
    lat_bias = ll[0, 0] - lat_slope * xy[2, 1]
    long_bias = ll[0, 1] - long_slope * xy[0, 0]

    print(f"Lat Slope: {lat_slope}, Long Slope: {long_slope}, Lat Bias: {lat_bias}, Long Bias: {long_bias}")
    lat_long_interpolation = np.zeros((len(points_targeted), 2))
    lat_long_interpolation[:, 0] = lat_slope * points_targeted[:, 1] + lat_bias
    lat_long_interpolation[:, 1] = long_slope * points_targeted[:, 0] + long_bias

    return lat_long_interpolation


def turning_points(xy_path_generated, lat_long_interpolation):  # using xy pixel data for more robustness ; not latitude & longitude
    """Turning point function for generated path
    This function takes in the generated planned path (in xy pixel coordinates) and the latitude and longitude interpolation values.
    If the waypoint within the numpy array sequence of the generated planned path is divisible by 4 with no remaineder, it is marked as 1
    within a turn list indicating this waypoint is a spline waypoint. All other waypoints not divisible by 4 are marked as 0.
     
    Args:
        xy_path_generated: numpy array of the generated planned waypoints in xy pixel data.
        lat_long_interpolation: array of latitude and longitude values that correspond to their respective pixel coordinates.
    
    Returns: 
        lat_long_turn: 3 column array containing the latitude and longitude values as well as the turning list for each respective lat and long value"""
    
    turn_list = np.zeros(len(xy_path_generated), dtype=int)

    for i in range(1, len(xy_path_generated)):
        if i % 4 == 1 or i / 4 == 0.25:  # 'acceleration' - way point
            turn_list[i] = 0
        if i % 4 == 2 or i / 4 == 0.5:  # 'deceleration' - waypoint
            turn_list[i] = 0
        if i % 4 == 3 or i / 4 == 0.75:  # non spline waypoint
            turn_list[i] = 0
        if i % 4 == 0:  # spline waypoint
            turn_list[i] = 1
    lat_long_turn = np.column_stack((lat_long_interpolation, turn_list))

    return lat_long_turn


def main():
    export = True

    # GUI Values - for debugging purposes
    # 4 reference points representing left, right, top and bottom of the image
    ref_pix = np.array([
                [342, 145],
                [827, 493],
                [523, 143],
                [378, 590],
    ])
    ref_lla = np.array([
                [29.408446, -82.170371],
                [29.407492, -82.168867],
                [29.408435, -82.169896],
                [29.407235, -82.170208]
    ])


    reference_points = np.column_stack((ref_pix, ref_lla))
    generated_path = [
        [397.96,  198.88],
        [397.96,  282.69],
        [397.96,  450.31],
        [397.96,  534.12],
        [435.27,  534.12],
        [435.27,  450.31],
        [435.27,  282.69],
        [435.27,  198.88],
        [472.58,  198.88],
        [472.58,  282.69],
        [472.58,  450.31],
        [472.58,  534.12],
        [509.88,  534.12],
        [509.88,  450.31],
        [509.88,  282.69],
        [509.88,  198.88],
        [547.19,  198.88],
        [547.19,  282.69],
        [547.19,  450.31],
        [547.19,  534.12],
        [ 584.5,  534.12],
        [ 584.5,  450.31],
        [ 584.5,  282.69],
        [ 584.5,  198.88],
        [621.81,  198.88],
        [621.81,  282.69],
        [621.81,  450.31],
        [621.81,  534.12],
        [659.12,  534.12],
        [659.12,  450.31],
        [659.12,  282.69],
        [659.12,  198.88],
        [696.42,  198.88],
        [696.42,  282.69],
        [696.42,  450.31],
        [696.42,  534.12],
        [733.73,  534.12],
        [733.73,  468.94],
        [733.73,  338.56],
        [733.73,  273.38],
        [771.04,  273.38],
        [771.04,  338.56],
        [771.04,  468.94],
        [771.04,  534.12],
    ]
    generated_path = np.array(generated_path)
    latitude_longitude_interpolation = bi_linear_interpolation(reference_points, generated_path)
    lat_long_reference = bi_linear_interpolation(reference_points, reference_points)

    print("Lat Long Interpolation: ", latitude_longitude_interpolation[:5])
    print("Lat Long Reference: ", lat_long_reference)

    plt.plot(reference_points[:, 0], -reference_points[:, 1], color='red', label='Reference Points')
    plt.plot(generated_path[:, 0], -generated_path[:, 1], color='blue', label='Generated Path')
    plt.title('Reference Points and Path in Pixel Space')
    plt.legend()
    plt.show()

    plt.plot(lat_long_reference[:, 1], lat_long_reference[:, 0], color='red', label='Transformed Reference Points')
    plt.plot(ref_lla[:, 1], ref_lla[:, 0], color='green', label='Reference Points')
    plt.plot(latitude_longitude_interpolation[:, 1], latitude_longitude_interpolation[:, 0], color='blue', label='Interpolated Points')
    plt.title('Reference Points and Path in Latitude and Longitude Space')
    plt.legend()
    plt.show()

    if export:

        lat_long_turn = turning_points(generated_path, latitude_longitude_interpolation)
        waypoints_data = plan_mission(lat_long_turn)  
        output_filename = 'Long_Core_Mission_Matrix22_test_0_turn_v12.waypoints'

        try:
            # Open the file in write mode
            with open(output_filename, 'w', encoding='UTF-8', newline='') as file:
                file.write('QGC WPL 110\n')

                for row in waypoints_data:
                    line = '\t'.join(map(str, row))  # Convert each element to string and join with tab
                    file.write(f"{line}\n")

            print(f'Waypoints successfully exported to {output_filename}')
        except Exception as e:
            print(f'Failed to export waypoints: {e}')

if __name__ == "__main__":
    main()