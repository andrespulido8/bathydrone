#!/usr/bin/env python3
""" Generate a complete coverage path for a side-scan sonar vessel on an arbitrary convex region.
    Author: Nicholas Sardinia
"""
import sys, os
import matplotlib.pyplot as plt
import geopandas as gpd
import numpy as np
import math
import pandas as pd
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
from shapely.prepared import prep
from py2opt.routefinder import RouteFinder

class PathPlanner:
    """ 
    The bathydrone path planner.

    Attributes: 
        self.bounding: BoundingRegion object to be pathed.
        self.sideScanAngle: angle of the side-scan sonar.
        self.maxRange: maximum range of the side-scan sonar.
        self.minRange: minimum range of the side-scan sonar.
        self.noRequireParallel: disable parallel pathing requirement.
    """

    def __init__(self, sideScanAngle=30, **system_params):
        self.bounding = None
        self.sideScanAngle = sideScanAngle
        if 'maxRange' in system_params:
            setattr(self, 'maxRange', system_params['maxRange'])
        if 'minRange' in system_params:
            setattr(self, 'minRange', system_params['minRange'])
        if 'maxDepth' in system_params:
            setattr(self, 'maxDepth', system_params['maxDepth'])
        if 'noRequireParallel' in system_params:
            setattr(self, 'noRequireParallel', system_params['noRequireParallel'])
    
    def PathRegion(self):
        """ Generate a complete coverage path for a side-scan sonar vessel on an arbitrary convex region. """
        # GENERATE POLYGON OBJECT (required for shapely)
        geom = Polygon(self.bounding.polygonVertices)
        xList, yList = list(zip(*self.bounding.polygonVertices))
        
        # APPROXIMATE DECOMPOSITION
        # Path distance approximation (when physical parameters are unknown)
        pathDist = abs((max(xList) - min(xList))/18)
        # Decomposition Tolerance Constant (SL-Concavity)
        tol = pathDist*1
        pTest = self.decomposePolygon(self, tol, geom)

        # GENERATE PATH FOR EACH DECOMPOSED POLYGON
        startPos = [min(xList), min(yList)]
        pathCenters, testPathArr = self.generatePathsOfDecomposedRegions(self, pTest, pathDist, [startPos])
        
        # 2-OPT HEURISTIC
        best_distance, best_route = self.orderTwoOpt(pathCenters)

        # PLOTTING
        self.plotPath(xList, yList, testPathArr, pathCenters, best_route, best_distance)

    def getBoundingPolygon(self, **boundingInput):
        """ Create a Bounding Polygon Object to Store in the PathPlanner. """
        #TODO add support for depth map
        #TODO add support for direct polygon input (array of tuples)
        if 'csvName' in boundingInput:
            #fetch from CSV
            setattr(self, 'bounding', boundingRegion(boundingInput['csvName']))
        if 'boundingPolygon' in boundingInput:
            #get from bounding polygon provided directly as an array of tuples.
            print("Please input a csv")

    def  get_max_range(self):
        """ Print the maximum range of the side-scan sonar. """
        if hasattr(self, 'maxRange'):
            print(self.maxRange) 
        else: 
            print("Maximum range was not set.")    

    @staticmethod
    def generatePath(self, polygonToPath, pathDist):
        """
        Input: polygonToPath - list of points in polygon 
                pathDist - distance between waypoints
        Output: chosenPath - list of points in path
                    bestPL - path length
                    emptyPath - flag for empty path
        """
        c2 = 0.4  # weight for number of turns
        lc = 100000000 #starting cost, very high to avoid initialization issues.
        #optimization for path orientation.
        #Tests angle in 10 degree increments. Returns all path lengths. 
        bestPL = 100000000  #best path length
        minTurns = 10000000  #minimum number of turns
        currPL = 0  #current path length
        pathLengths = []
        
        #instead of gridding, test with linear distance to refine (max linear dist)
        Bestgeom = Polygon(polygonToPath)
        Bestgrid = self.partition(self, Bestgeom, pathDist)
        for testAngle in range(0, 180, 30):
            polygonToPathRotated =  []      

            #rotation matrix
            transformAngle = [[np.cos(np.deg2rad(testAngle)), -np.sin(np.deg2rad(testAngle))], 
                        [np.sin(np.deg2rad(testAngle)), np.cos(np.deg2rad(testAngle))]]
            
            #do the inverse rotation for each point in dome
            for point in polygonToPath:
                    rotatedpoint = list((np.dot(transformAngle, point)) )
                    polygonToPathRotated.append(rotatedpoint)

            geom = Polygon(polygonToPathRotated)
            grid = self.partition(self, geom, pathDist)
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

                if lc > currC:
                    chosenPath = path
                    lc = currC
                    Bestgrid = grid
                    Bestgeom = geom
                
                if bestPL > currPL:
                    bestPL = currPL

                if minTurns > numTurns:
                    minTurns = numTurns
            else:
                bestPL = 0
                chosenPath = []
                emptyPath = 1
            
        return chosenPath, bestPL, emptyPath, Bestgrid, Bestgeom

    @staticmethod
    def grid_bounds(geom, delta):
        """ 
        Input: geom (shapely polygon), delta (distance between path lines)
        Output: grid boundaries from which to draw path.
        """

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
    
    @staticmethod
    def partition(self, geom, delta):
        """ Define a grid of cells for a polygon."""
        prepared_geom = prep(geom)
        grid = list(filter(prepared_geom.covers, self.grid_bounds(geom, delta)))
        return grid
    
    @staticmethod    
    def listConvexHull(polyPoints):
        """ Find the convex hull of a polygon."""
        cHull = ConvexHull(polyPoints)
        listHull = []
        for i in range(len(cHull.vertices)):
            listHull.append(polyPoints[cHull.vertices[i]])
        return listHull
    
    @staticmethod
    def concavityChecker(polyPoints, polyGeom):
        """ Take a polygon, and export an inorder list of SL-concavities"""
        
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
    
    @staticmethod
    def resolveConvex(polyPoints, rIndex, concavities):
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
    
    @staticmethod    
    def makeConvexRec(self, polyPoints, tolerance, polyStore, polyPointsGeom): 
        """ Take a polygon and apply approximate convex decomposition.

            Source: Lien et.al. "Approximate Convex Decomposition of Polygons" 
            https://www.sciencedirect.com/science/article/pii/S0925772105001008
        """
        concavityList = self.concavityChecker(polyPoints, polyPointsGeom)
        t = max(concavityList)
        if t <= tolerance:
            if polyPoints not in polyStore:
                polyStore.append(polyPoints)
            return polyPoints
        else:
            #Don't do this more than once per recursion
            ps1, ps2, polyGeom1, polyGeom2 = self.resolveConvex(polyPoints, concavityList.index(max(concavityList)), concavityList)
            self.makeConvexRec(self, ps1, tolerance, polyStore, polyGeom1)
            self.makeConvexRec(self, ps2, tolerance, polyStore, polyGeom2)
    
    @staticmethod
    def makeConvex(self, polyPoints, tolerance, polyPointsGeom):
        """ apply approximate convex decompsition to a simple polygon """
        ps = []
        self.makeConvexRec(self, polyPoints, tolerance, ps, polyPointsGeom)
        return ps
    
    @staticmethod
    def getPolygonFromCSVExternal(csvName):
        """ get region of interest from csv (edge detection) """
        xList = []
        yList = []
        file_path = os.path.join(os.getcwd(), "csv", csvName)
        df = pd.read_csv(file_path, usecols=["Latitude", "Longitude"])
        yList = df.Latitude.values.tolist()
        xList = df.Longitude.values.tolist()

        return xList, yList
    
    @staticmethod
    def generatePathsOfDecomposedRegions(self, pTest, pathDistOrig, startPoint):
        """ generate a coverage path of a set of approximately convex polygons """
        testPathArr = []
        pathCenters = startPoint
        variablePathDist = pathDistOrig
        for i in range(len(pTest)):
            x1, y1 = zip(*pTest[i])
            maxArea = (max(x1)-min(x1))*(max(y1)-min(y1))
            if (maxArea > (5*pathDistOrig*pathDistOrig)):
                genPath = self.generatePath(self, pTest[i], variablePathDist)
                if (len(genPath[0]) > 1):
                    testPathArr.append(genPath[0])
                    xCenter, yCenter = zip(*genPath[0])
                    xLoc = (max(xCenter) + min(xCenter))/2
                    yLoc = (max(yCenter) + min(yCenter))/2
                    pathCenters.append([xLoc, yLoc])

        return pathCenters, testPathArr

    @staticmethod
    def decomposePolygon(self, tolerance, polygonToDecompose):
        """ decompose polygon in to approximately convex sub-polygons """
        poly = gpd.GeoSeries([polygonToDecompose])
        polyCoords = poly.get_coordinates()
        polyCoordsList = list(polyCoords.itertuples(index=False, name=None))
        decomposedPolygon = self.makeConvex(self, polyCoordsList, tolerance, poly)
        return decomposedPolygon

    @staticmethod
    def orderTwoOpt(pathCenters):
        """ approximate convex decomposition tsp solution with 2-opt algorithm """
        pathLocs = []
        for i in range(len(pathCenters)):
            pathLocs.append(str(i))

        distMat = []
        for i in range(len(pathCenters)):
            currMat = []
            for j in range(len(pathCenters)):
                currMat.append(math.dist(pathCenters[i], pathCenters[j]))
            distMat.append(currMat)

        initDist = 0
        for i in range(len(pathCenters)-1):
            initDist += distMat[i][i+1]
        
        # 2-OPT ITERATION (from py2opt libary, stable)
        
        # stops route_finder from printing timer    
        sys.stdout = open(os.devnull, 'w')

        route_finder = RouteFinder(distMat, pathLocs, iterations=5)
        best_distance, best_route = route_finder.solve()

        # re-enables command-line prints
        sys.stdout = sys.__stdout__
        
        return best_distance, best_route
    
    @staticmethod
    def plotPath(xList, yList, testPathArr, pathCenters, best_route, best_distance):
        """ generate matplotlib plot of the competed path """
        # PLOTTING SETUP
        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        ax1.axes.get_xaxis().set_visible(False)
        ax1.axes.get_yaxis().set_visible(False)


        # PLOTTING PATHS
        #print(testPathArr)
        ax1.plot(xList, yList, c='black', marker='o', markersize='0.25')
        for i in range(len(testPathArr)):
            x1, y1 = zip(*testPathArr[i])
            ax1.plot(x1, y1, c='green', marker='o', markersize='0.5', zorder=1)
        
        # PLOTTING VISIT ORDER
        j = 0
        sSize = abs(abs(max(xList))-abs((min(xList)))) * abs(abs(max(yList))-abs((min(yList))))*50000
        xc2, yc2 = zip(*pathCenters)


        for xIm, yIm in zip(xc2, yc2):
            ax1.text(xIm, yIm, str(j), color='grey', fontsize=12, fontweight="bold", zorder=3)
            #plt.scatter(xIm, yIm, s=sSize, facecolors='white', edgecolors='grey', zorder=2)
            j += 1

        # SHOW PLOT
        plt.axis('equal')
        plt.show()




    @staticmethod
    def plotPath3D(xList, yList, testPathArr, pathCenters):
        """ Generate 3D plot of the path and depthmap. """

        #TODO add depth map
        # PLOTTING SETUP
        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        ax1.axes.get_xaxis().set_visible(False)
        ax1.axes.get_yaxis().set_visible(False)

        #3d plotting
        ax = plt.axes(projection='3d')

        # 3D bounding polygon
        lakeZero = 6
        zdata = [lakeZero] * len(xList) 
        ax.plot3D(xList, yList, zdata, color="green")

        # 3D Approximate Depth Map
        # Initial guess = concentric rings increasing depth towards center.
        # Create an array of depths and plot it in 3d.
        # Calculate the average center
        avg_x = sum(xList) / len(xList)
        avg_y = sum(yList) / len(yList)

        # Create the (x, y) tuple
        avg_center = (avg_x, avg_y)
        # Generate concentric rings
        max_depth = 5
        num_rings = 4
        max_radius = (max(xList) - min(xList))/2.25
        depths = np.linspace(0, max_depth, num_rings)
        ring_radius = np.linspace(0, max_radius, num_rings)

        for depth, radius in zip(depths, ring_radius):
            theta = np.linspace(0, 2*np.pi, 100)
            x = avg_center[0] + radius * np.cos(theta)
            y = avg_center[1] + radius * np.sin(theta)
            z = np.full_like(x, depth)
            ax.plot3D(x, y, z, color='blue')

        for i in range(len(testPathArr)):
            x1, y1 = zip(*testPathArr[i])
            ax.plot3D(x1, y1, lakeZero, c='black', marker='o', markersize='0.5', zorder=1)
        
        plt.show()
    
#TODO add manipulations to bounding region polygon.
class boundingRegion:
    """ 
    The bounding polygon for the region of interest.

    Attributes: 
        polygonVertices (list of tuples): vertices forming exterior of the bounding polygon.
    """
    
    def __init__(self, csvName):
        self.polygonVertices = self.getPolygonFromCSV(csvName)

    @staticmethod
    def getPolygonFromCSV(csvName):
        """ get region of interest from csv (manual) """
        file_path = os.path.join(os.getcwd(), "csv", csvName)
        df = pd.read_csv(file_path, usecols=["Latitude", "Longitude"])
        return list(zip(df.Longitude.values.tolist(), df.Latitude.values.tolist()))

    def printBoundingPolygon(self):
        print(self.polygonVertices)
    
def main():
    """
    Example usage of the PathPlanner class.

    1. Create a PathPlanner object.
    2. Set the bounding polygon from csv.
    3. Generate and display a coverage path for the region.
    """
    myPlanner = PathPlanner(45, maxRange=1000)
    myPlanner.getBoundingPolygon(csvName="newnans-lake-coords.csv")
    myPlanner.PathRegion()
    return 0

if __name__ == "__main__":
    main()