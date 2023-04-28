'''
Simple Python script for demonstrating water body edge detection
pipeline and conversion of water perimeter points into latitude and
longitude coordinates.
'''
#!/usr/bin/env python3

import csv
import numpy as np
import cv2
import grip

def find_xy_bounds(contour):
    '''
    Finds and returns the minimum and maximum value of
    the x and y coordinates of all points in a contour.
    '''

    # find min and max x and y of all contour points
    # assumes pixel coordinates are (x, y) = (column, row)
    # with (0, 0) as the top-left corner of the image
    min_x = contour[0][0][0]
    max_x = contour[0][0][0]
    min_y = contour[0][0][1]
    max_y = contour[0][0][1]

    for point in contour:
        min_x = min(point[0][0], min_x)
        max_x = max(point[0][0], max_x)
        min_y = min(point[0][1], min_y)
        max_y = max(point[0][1], max_y)

    return min_x, max_x, min_y, max_y


def xy_to_latlong(latlong_bounds, contour):
    '''
    Converts a contour's list of (x, y) coordinates into
    latitude and longitude.
    '''

    left_x, right_x, top_y, bottom_y = find_xy_bounds(contour)

    latlong_points = []

    for point in contour:
        point_scaled = [
                (point[0][0]-left_x)/(right_x-left_x),
                (point[0][1]-top_y)/(bottom_y-top_y)]

        nnl_lat_bottom  = latlong_bounds["lat"][0]
        nnl_lat_top     = latlong_bounds["lat"][1]
        nnl_long_left   = latlong_bounds["long"][0]
        nnl_long_right  = latlong_bounds["long"][1]

        nnl_latlong_height  = nnl_lat_top - nnl_lat_bottom
        nnl_latlong_width   = nnl_long_right - nnl_long_left

        point_latlong = [
                nnl_long_left + point_scaled[0]*nnl_latlong_width,
                nnl_lat_top - point_scaled[1]*nnl_latlong_height]

        latlong_points.append(point_latlong)

    return latlong_points

def main():
    '''
    Main function to demonstrate OpenCV pipeline functionality.

    '''

    # Latitude and longitude values for a bounding box around Newnans Lake.
    # Points are arranged South-to-North and East-to-West respectively.
    nnl_latlong = {
            "lat" : (29.613571, 29.678637),
            "long" : (-82.252861, -82.198316)
    }

    img = cv2.imread("Figures/newnans-lake-1.png")

    pipeline = grip.Pipeline()
    pipeline.process(img)
    filtered_contours = pipeline.filter_contours_output

    nnl_contour = filtered_contours[0]
    print('Newnans Lake:')
    print(f'Type: {type(nnl_contour)}')
    print(f'Size: {len(nnl_contour)}')
    print(nnl_contour)
    print()

    latlong_output = xy_to_latlong(nnl_latlong, nnl_contour)


    with open('water-body-demo-out.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)

        writer.writerow(['Latitude', 'Longitude'])
        for point in latlong_output:
            writer.writerow([point[1], point[0]])

    cv2.drawContours(img, filtered_contours, -1, (0, 255, 0), 3)
    cv2.namedWindow("img", cv2.WINDOW_KEEPRATIO)
    cv2.imshow("img", img)
    cv2.resizeWindow("img", 2000, 2000)
    cv2.waitKey(-1)


if __name__ == "__main__":
    main()
