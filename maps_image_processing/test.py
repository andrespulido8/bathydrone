#!/usr/bin/env python3

import cv2
import numpy as np

def main():
    img = cv2.imread("map.png")
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([100, 0, 0], np.uint8)
    upper = np.array([220, 255, 255], np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    mask2 = np.dstack((mask,mask,mask))
    print(np.shape(mask2))
    print(np.shape(img))
    hori = np.concatenate((img,mask2), axis=0)

    kernel = np.ones((5,5), np.uint8)
    contouredmask = mask.copy()
    # mask = cv2.Canny(mask, 30, 200)
    contouredmask = cv2.erode(contouredmask,kernel, iterations=1)
    contouredmask = cv2.dilate(contouredmask,kernel, iterations=1)
    contours, heirarchy = cv2.findContours(contouredmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # to find out what the biggest area is
    areas = [cv2.contourArea(c) for c in contours]
    print(max(areas))

    # pick the biggest contour
    filteredcontours = [c for c in contours if cv2.contourArea(c) > 300000]
    cv2.drawContours(img, filteredcontours, -1, (0, 255, 0), 3)


    cv2.namedWindow("img", cv2.WINDOW_KEEPRATIO)
    cv2.imshow("img",img)
    cv2.resizeWindow("img", 2000, 2000)
    cv2.waitKey(-1)


if(__name__ == "__main__"):
    main()
