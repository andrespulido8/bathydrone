#!/usr/bin/env python3
""" Here add a description of the script
    Author: Nicholas Sardinia
"""
import os
import matplotlib.pyplot as plt
import geopandas as gpd
import matplotlib.path as pth
import numpy as np
import math
import csv
import pandas as pd
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.prepared import prep


def fullPath(xIm, yIm):
    #print(xIm)
    #print(yIm)
    polygonToPath = list(zip(xIm, yIm))
    
    pathDist = abs((max(xIm) - min(xIm))/15)
    geom = Polygon(polygonToPath)
    
    #APPROXIMATE CONVEX DECOMPOSITION
    tol = pathDist*1.5
    poly = gpd.GeoSeries([geom])
    polyCoords = poly.get_coordinates()
    polyCoordsList = list(polyCoords.itertuples(index=False, name=None))
    pTest = makeConvex(polyCoordsList, tol, poly)

    pathDistOrig = pathDist

    testPathArr = []
    #print(testPathArr)
    for i in range(len(pTest)):
        x1, y1 = zip(*pTest[i])
        #pathDist = abs((max(x1) - min(x1))/15)
        #Maximum area heuristic
        maxArea = (max(x1)-min(x1))*(max(y1)-min(y1))
        if (maxArea > (5*pathDistOrig*pathDistOrig)):
            genPath = generatePath(pTest[i], pathDist/3.5)
            if (len(genPath[0]) > 2):
                testPathArr.append(genPath[0])
                #pathLengthTot += genPath[1]
                
    finalPath = []
    #print(testPathArr[0])

    if (len(testPathArr) == 1):
        #print("triggered")
        for j in range(len(testPathArr[0])):
            finalPath.append(testPathArr[0][j])
    else:
        for i in range(len(testPathArr)):
            x1, y1 = zip(*testPathArr[i])
            if (i == len(testPathArr)-1):
                for j in range(len(testPathArr[i])):
                    #finalPath.append([testPathArr[i][j][0], finalPath[len(finalPath)-1][1]])
                    finalPath.append(testPathArr[i][j])
            else:
                for j in range(len(testPathArr[i])):
                        #finalPath.append([testPathArr[i][j][0], finalPath[len(finalPath)-2][1]])
                        finalPath.append(testPathArr[i][j])
                x2, y2 = zip(*testPathArr[i+1])
    #print(finalPath)
    xList, yList = zip(*finalPath)

    waypoint_coords = []
    for (x, y) in zip(xList, yList):
        waypoint_coords.append(x)
        waypoint_coords.append(y)
    return waypoint_coords

def generatePath(polygonToPath, pathDist):
    """ TODO: document this function
    Input: polygonToPath - list of points in polygon 
              pathDist - distance between waypoints
    Output: chosenPath - list of points in path
                bestPL - path length
                emptyPath - flag for empty path
    """
    c1 = 0.9  # weight for path length
    c2 = 0.4  # weight for number of turns
    lc = 100000000
    #optimization for path orientation.
    #Tests angle in 10 degree increments. Returns all path lengths. 
    bestPL = 100000000  #best path length
    minTurns = 10000000  #minimum number of turns
    currPL = 0  #current path length
    pathLengths = []
    savedNumTurns = 10000
    savedPL = 100000
    
    #instead of gridding, test with linear distance to refine (max linear dist)
    for testAngle in range(0, 180, 10):
        polygonToPathRotated =  []      

        #rotation matrix
        transformAngle = [[np.cos(np.deg2rad(testAngle)), -np.sin(np.deg2rad(testAngle))], 
                      [np.sin(np.deg2rad(testAngle)), np.cos(np.deg2rad(testAngle))]]
        
        #do the inverse rotation for each point in dome
        for point in polygonToPath:
                rotatedpoint = list((np.dot(transformAngle, point)) )
                polygonToPathRotated.append(rotatedpoint)

        geom = Polygon(polygonToPathRotated)
        grid = partition(geom, pathDist)
        path = []
        manipulableGrid = list(grid)
        emptyPath = 0

        if len(manipulableGrid) >= 2:
            emptyPath = 0
            #PATHING THE GRID
            #Set direction marker
            #DIR = 1, UP
            dir = 1

            #1. Start at the grid coordinate with minimum X
            #Find minimum X position of the grid
            xx, yy = manipulableGrid[0].exterior.coords.xy
            minX = xx[0]
            width = abs(xx[0] - xx[2])
            
            #2. get slice of all grid squares with this X.
            #3. Choose what to do based on the details of the slice.
            xtest, ytest = manipulableGrid[len(manipulableGrid)-1].exterior.coords.xy
            maxX = xtest[0]


            numTurns = 0
            currX = minX
            currDex = 0
            while (currX <= maxX):
                slice = []
                for i in range(len(manipulableGrid)):
                    testX, testY = manipulableGrid[i].exterior.coords.xy
                    if (testX[0] == currX):
                        slice.append(manipulableGrid[i])
                        currDex = i
                
                if (len(slice) > 0):
                    if (dir == 1):
                        numTurns += 1
                        xx, yy = slice[0].exterior.coords.xy
                        firstPoint = []
                        firstPoint.append((xx[0]+xx[2])/2)
                        firstPoint.append((yy[0]+yy[2])/2)
                        xx2, yy2 = slice[len(slice)-1].exterior.coords.xy
                        endPoint = [] 
                        endPoint.append((xx2[0]+xx2[2])/2)
                        endPoint.append((yy2[0]+yy2[2])/2)
                        path.append(firstPoint)
                        path.append(endPoint)
                    if (dir == -1):
                        numTurns += 1
                        xx, yy = slice[len(slice)-1].exterior.coords.xy
                        firstPoint = []
                        firstPoint.append((xx[0]+xx[2])/2)
                        firstPoint.append((yy[0]+yy[2])/2)
                        xx2, yy2 = slice[0].exterior.coords.xy
                        endPoint = [] 
                        endPoint.append((xx2[0]+xx2[2])/2)
                        endPoint.append((yy2[0]+yy2[2])/2)
                        path.append(firstPoint)
                        path.append(endPoint)
                if (currDex + 1 != len(manipulableGrid)):
                    currDex = currDex + 1
                    xDex, yDex = manipulableGrid[currDex].exterior.coords.xy
                    currX = xDex[0]
                    dir = dir * (-1)
                else:
                    currDex = currDex + 1
                    currX = currX + width
                    dir = dir * (-1)

                
            #rotate the chosen path back to the real orientation
            transformAngleRev = [[np.cos(np.deg2rad(-testAngle)), -np.sin(np.deg2rad(-testAngle))], 
                        [np.sin(np.deg2rad(-testAngle)), np.cos(np.deg2rad(-testAngle))]]
        
            xPath1, yPath2 = zip(*path)
        
            rotatedPath = []
            for point in path:
                rotatedpoint = list((np.dot(transformAngleRev, point)) )
                rotatedPath.append(rotatedpoint)

            path = rotatedPath
            totalLength = 0
            xConv, yConv = zip(*path)
            distTemp = 0
            for j in range(len(xConv)-2):
                distX = ((xConv[j])-(xConv[j+1]))*((xConv[j])-(xConv[j+1]))*69*69
                distY = ((yConv[j])-(yConv[j+1]))*((yConv[j])-(yConv[j+1]))*54.6*54.6
                distTemp += math.sqrt(distX + distY)
            totalLength += distTemp
            currPL = totalLength
            currC = 0.001*currPL + c2*numTurns
            pathLengths.append(totalLength)   

            if numTurns < minTurns:
                chosenPath = path
                lc = currC
                savedPL = currPL
                minTurns = numTurns
                
            
            if bestPL > currPL:
                bestPL = currPL

            if minTurns > numTurns:
                minTurns = numTurns

           #print("iter\n")
        else:
            bestPL = 0
            chosenPath = []
            emptyPath = 1

        
    return chosenPath, bestPL, emptyPath

def grid_bounds(geom, delta):
    """ Define a grid of cells for a polygon."""
    minx, miny, maxx, maxy = geom.bounds
    nx = int((maxx - minx)/delta)
    ny = int((maxy - miny)/delta)
    gx, gy = np.linspace(minx,maxx,nx), np.linspace(miny,maxy,ny)
    grid = []
    for i in range(len(gx)-1):
        for j in range(len(gy)-1):
            poly_ij = Polygon([[gx[i],gy[j]],[gx[i],gy[j+1]],[gx[i+1],gy[j+1]],[gx[i+1],gy[j]]])
            grid.append( poly_ij )
    return grid

def partition(geom, delta):
    """ Define a grid of cells for a polygon."""
    prepared_geom = prep(geom)
    gridFirst = list(filter(prepared_geom.intersects, grid_bounds(geom, delta)))
    grid = list(filter(prepared_geom.covers, grid_bounds(geom, delta)))
    return grid
    
def listConvexHull(polyPoints):
    """ Find the convex hull of a polygon."""
    cHull = ConvexHull(polyPoints)
    listHull = []
    for i in range(len(cHull.vertices)):
        listHull.append(polyPoints[cHull.vertices[i]])
    #print(listHull)
    return listHull

def concavityChecker(polyPoints, polyGeom):
    """ Take a polygon, and export an inorder list of SL-concavities"""
    #print("iteration")
    
    conHullC = polyGeom.convex_hull.get_coordinates()
    conHull = list(conHullC.itertuples(index=False, name=None))

    concavities = [0]*len(polyPoints)
    for i in range(len(polyPoints)):
        if polyPoints[i] in conHull:
            concavities[i] = 0
        else:
            #find bounding hull points
            if i == 0:
                j = len(polyPoints)-1
            else:
                j = i
   
            while (j >= 0):
                j = j-1
                if polyPoints[j] in conHull:
                    b1 = polyPoints[j]
                    break
            j = i
            if(i == (len(polyPoints)-1)):
                j = 0
            while (j < len(polyPoints)-1):
                j = j + 1
                if polyPoints[j] in conHull:
                    b2 = polyPoints[j]
                    break
            #Find perpendicular distance from line b1->b2 to point polyPoints[i]
            concavities[i] = abs((b1[0]-b2[0])*(b1[1]-polyPoints[i][1])-(b1[1]-b2[1])*(b1[0]-polyPoints[i][0]))/math.sqrt(math.pow(b1[0]-b2[0], 2)+math.pow(b1[1]-b2[1], 2))
    return concavities

def resolveConvex(polyPoints, rIndex, polyPointsGeom, concavities):
    """ Resolve a convex polygon into two polygons."""
    #TODO improve optimization

    #tunable constants for heuristic 
    ignoreRange = [rIndex + len(concavities) / 5.0, rIndex - len(concavities)/5.0]
    sc = 0.1
    sd = 1
    #Find split points
    bestScore = 0
    bestDex = 0
    for i in range(len(polyPoints)):
        #Check unless in ignore range
        if (i >= ignoreRange[0] or i <= ignoreRange[1]) and abs(sd*math.dist(polyPoints[rIndex], polyPoints[i])) != 0.0:
            score = (1+sc*concavities[i])/(abs(sd*math.dist(polyPoints[rIndex], polyPoints[i])))
            if score > bestScore:
                bestScore = score
                bestDex = i
    poly1 = []
    poly2 = polyPoints.copy()
    if rIndex < bestDex:
        j = rIndex
        while j <= bestDex:
            poly1.append(polyPoints[j])
            if j != rIndex and j != bestDex:
                poly2[j] = (-100000, -100000)
            j = j + 1
        while (-100000, -100000) in poly2:
            poly2.remove((-100000, -100000))
        poly1.append(polyPoints[rIndex])

    if rIndex > bestDex:
        j = bestDex
        while j <= rIndex:
            poly1.append(polyPoints[j])
            if j != rIndex and j != bestDex:
                poly2[j] = (-100000, -100000)
            j = j + 1
        while (-100000, -100000) in poly2:
            poly2.remove((-100000, -100000))
        poly1.append(polyPoints[bestDex])

    polygonUse1 = Polygon(poly1)
    polygonUse2 = Polygon(poly2)
    polyGeom1 = gpd.GeoSeries([polygonUse1])
    polyGeom2 = gpd.GeoSeries([polygonUse2])

    return poly1, poly2, polyGeom1, polyGeom2
    
def makeConvexRec(polyPoints, tolerance, polyStore, polyPointsGeom): 
    """ Take a polygon and apply approximate convex decomposition.

        Source: Lien et.al. "Approximate Convex Decomposition of Polygons" 
        https://www.sciencedirect.com/science/article/pii/S0925772105001008
    """
    concavityList = concavityChecker(polyPoints, polyPointsGeom)
    t = max(concavityList)
    if t <= tolerance:
        if polyPoints not in polyStore:
            polyStore.append(polyPoints)
        return polyPoints
    else:
        #Don't do this more than once per recursion
        ps1, ps2, polyGeom1, polyGeom2 = resolveConvex(polyPoints, concavityList.index(max(concavityList)), polyPointsGeom, concavityList)
        makeConvexRec(ps1, tolerance, polyStore, polyGeom1)
        makeConvexRec(ps2, tolerance, polyStore, polyGeom2)

def makeConvex(polyPoints, tolerance, polyPointsGeom):
    """ Take a polygon and apply approximate convex decomposition."""
    ps = []
    makeConvexRec(polyPoints, tolerance, ps, polyPointsGeom)
    return ps

def main():
    #Get polygon from edge detection
    xList = []
    yList = []

    file_name = "newnans-lake-coords.csv"
    file_path = os.path.join(os.getcwd(), "csv", file_name)
    df = pd.read_csv(file_path, usecols=["Latitude", "Longitude"])
    yList = df.Latitude.values.tolist()
    xList = df.Longitude.values.tolist()
    
    output = fullPath(xList, yList)
    print(output)
    
    #plot
    
    
    
    return 0

if __name__ == "__main__":
    main()