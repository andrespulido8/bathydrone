'''
Simple Python script for demonstrating water body edge detection
pipeline and conversion of water perimeter points into latitude and
longitude coordinates.
'''
#!/usr/bin/env python3
import numpy as np
import math
import csv  # for writing out coordinates to a CSV file
import cv2  # OpenCV python library
import grip # GRIP-generated pipeline (see grip.py)

def find_bounding_points(contour):
    '''
    Finds and returns the leftmost, rightmost, top, and bottom
    points in the input contour, in that order.

    Note that (x, y) pixel coordinates increase down and right
    with (0, 0) as the top-left corner.
    '''

    left    = contour[0]
    right   = contour[0]
    top     = contour[0]
    bottom  = contour[0]

    for point in contour:
        if point[0][0] < left[0][0]:
            left = point
        elif point[0][0] > right[0][0]:
            right = point

        if point[0][1] < top[0][1]:
            top = point
        elif point[0][1] > bottom[0][1]:
            bottom = point

    return left, right, top, bottom

def xy_to_latlong(latlong_bounds, xy_bounds, contour):
    '''
    Converts a contour's list of (x, y) coordinates into
    latitude and longitude.
    '''

    left_x      = xy_bounds[0][0][0]
    right_x     = xy_bounds[1][0][0]
    top_y       = xy_bounds[2][0][1]
    bottom_y    = xy_bounds[3][0][1]

    latlong_points = []

    nnl_lat_bottom  = latlong_bounds["lat"][0]
    nnl_lat_top     = latlong_bounds["lat"][1]
    nnl_long_left   = latlong_bounds["long"][0]
    nnl_long_right  = latlong_bounds["long"][1]

    for point in contour:
        point_scaled = [
                (point[0][0]-left_x)/(right_x-left_x),
                (point[0][1]-top_y)/(bottom_y-top_y)]

        nnl_latlong_height  = nnl_lat_top - nnl_lat_bottom
        nnl_latlong_width   = nnl_long_right - nnl_long_left

        point_latlong = [
                nnl_long_left   + point_scaled[0]*nnl_latlong_width,
                nnl_lat_top     - point_scaled[1]*nnl_latlong_height]

        latlong_points.append(point_latlong)

    return latlong_points


def scale_contour(num_points, contour):
    '''
    Scales a contour down to the specified number of points.
    '''
    # how often to take another value from the input contour
    step_size = int(math.ceil((len(contour)/num_points)))

    # use slicing to take every n-th item
    new_contour = contour[0::step_size]
    if len(new_contour) < len(contour):
        new_contour = np.append(new_contour, [contour[len(contour)-1]], axis=0)

    return new_contour
    #return np.ndarray(shape=(num_points, 1, 2), buffer=np.array(new_contour))

def draw_contour(img, contour):
    '''
    Draws a contour on the input image.
    '''
    cv2.drawContours(img, [contour], 0, (0, 255, 0), 3)

def draw_points(img, points):
    '''
    Draws a list of (x, y) points on the input image.
    '''

    for point in points:
        cv2.circle(img, point[0], radius=8, color=(255, 50, 50), thickness=-1)


def display_image(img):
    '''
    Displays the input image, then waits for the
    user to press any key.
    '''
    cv2.namedWindow('img', cv2.WINDOW_KEEPRATIO)
    cv2.imshow('img', img)
    cv2.resizeWindow('img', 2000, 2000)
    while cv2.getWindowProperty('img', cv2.WND_PROP_VISIBLE) >= 1:
        key = cv2.waitKey(10)
        if key != -1:
            cv2.destroyWindow('img')
            break

def main():
    '''
    Main function to demonstrate OpenCV pipeline functionality.
    '''

    # Read in the test image of Newnans Lake.
    img = cv2.imread("Figures/newnans-lake-1.png")

    # Run the image through the pipeline.
    pipeline = grip.Pipeline()
    pipeline.process(img)
    filtered_contours = pipeline.filter_contours_output

    # Take the first contour from the list (which, in our case, is the
    # Newnans Lake contour).
    nnl_contour = filtered_contours[0]
    print(f'Newnans Lake contour generated with {len(nnl_contour)} points.')
    print()

    # Find (x, y) points bounding Newnans Lake.
    nnl_xy = find_bounding_points(nnl_contour)

    # Prompt the user to collect lat/long bounding points
    print('Manual collection of bounding latitude/longitude points is required.')

    collect_latlong = input('Would you like to collect new points now? (y/N): ')
    if collect_latlong == 'y':
        print('Please collect latitudes of top and bottom points on Newnans Lake,\n\
                as well as longitudes of left and right points (highlighted blue),\n\
                using Google Maps or a similar tool. Then add these to the input\n\
                file \'newnans-lake-bounds.csv\' and restart this script.')
        input('Press Enter to see the points drawn on the image. Press any key to\n\
                close the image when you are finished.')

        draw_contour(img, nnl_contour)
        draw_points(img, nnl_xy)
        display_image(img)

        return

    # Latitude and longitude values for a bounding box around Newnans Lake.
    # Points are arranged South-to-North and East-to-West respectively.
    # Points are read in from input file.
    with open('newnans-lake-bounds.csv', 'r', encoding='UTF-8', newline='') as csvfile:
        reader = csv.reader(csvfile)

        for row in reader:
            if row[0] == ('Latitude'):
                lat_coords = row
            elif row[0] == ('Longitude'):
                long_coords = row

        nnl_latlong = {
                "lat"   : (float(lat_coords[1]), float(lat_coords[2])),
                "long"  : (float(long_coords[1]), float(long_coords[2]))
        }

    # make sure the lat/long points were pulled in successfully
    assert nnl_latlong

    output_file_name = 'newnans-lake-coords.csv'
    print(f'Not collecting lat/long bounds. Output will be sent to {output_file_name}.')

    num_latlong_points = int(input("Please enter the number of points you would like to output: "))
    nnl_contour = scale_contour(num_latlong_points, nnl_contour)
    print(f'resulting contour has {len(nnl_contour)} points.')

    latlong_output = xy_to_latlong(nnl_latlong, nnl_xy, nnl_contour)

    # Write lat/long coordinates to output CSV file.
    with open(output_file_name, 'w', encoding='UTF-8', newline='') as csvfile:
        writer = csv.writer(csvfile)

        writer.writerow(['Latitude', 'Longitude'])
        for point in latlong_output:
            writer.writerow([point[1], point[0]])

    draw_contour(img, nnl_contour)
    display_image(img)

if __name__ == "__main__":
    main()
