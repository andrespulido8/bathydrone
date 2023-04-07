#!/usr/bin/env python3

import cv2
import numpy as np

import grip

#def xy_to_latlong(ref_coords): 
#    return None

def main():

    ## (x, y) : (lat, long)
    #newnans_1_reference_coords = {
    #        (791, 373) : (29.679634, 82.254944),
    #        (1182, 908) : (29.612946, 82.198982)
    #}

    img = cv2.imread("Figures/newnans-lake-1.png")
    
    pipeline = grip.Pipeline()
    pipeline.process(img)
    filtered_contours = pipeline.filter_contours_output

    print(type(filtered_contours))
    print(len(filtered_contours))

    cv2.drawContours(img, filtered_contours, -1, (0, 255, 0), 3)


    cv2.namedWindow("img", cv2.WINDOW_KEEPRATIO)
    cv2.imshow("img",img)
    cv2.resizeWindow("img", 2000, 2000)
    cv2.waitKey(-1)


if(__name__ == "__main__"):
    main()
