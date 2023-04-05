#!/usr/bin/env python3
"""
Updated algorithm to generate waypoints for convex polygons 
"""

import matplotlib.pyplot as plt
import numpy as np
import math

class HardwareSpecs:
    """ Define the boat hardware"""
    # accounts for radius of curvature and side-scan sonar geometry
    def __init__(self, D, alpha_2):
        self.rho = D*np.tan(alpha_2)
        # rho - radius that constrains the spacing between each
        # D - estimated depth of water
        # alpha_2 = Measured angle related to coverage of side-scan sonar

class pathPlanner:
    """ Generates path, starting from lower-left corner. 
    Lawnmower lines for full convex coveragae.
    Returns path as a series of grid points from bottom left to top right
    """
    def __init__(self, pointsX, pointsY, rho):
        self.pointsX = pointsX
        self.pointsY = pointsY
        self.rho = rho
        self.pathPointsX = []
        self.pathPointsY = []
        self.pathX = []
        self.pathY = []

        #find maximum X
        maxXDex = 0
        maxX = self.pointsX[0]
        for i in range(0, len(self.pointsX)):
            if self.pointsX[i] > maxX:
                maxX = self.pointsX[i]
                maxXDex = i

        minXDex = 0
        minX = self.pointsX[0]
        for i in range(0, len(self.pointsX)):
            if self.pointsX[i] < minX:
                minX = self.pointsX[i]
                minXDex = i
        
        maxYDex = 0
        maxY = self.pointsY[0]
        for i in range(0, len(self.pointsY)):
            if self.pointsY[i] > maxY:
                maxY = self.pointsY[i]
                maxYDex = i
        
        minYDex = 0
        minY = self.pointsY[0]
        for i in range(0, len(self.pointsY)):
            if self.pointsY[i] < minY:
                minY = self.pointsY[i]
                minYDex = i
         

        #iterate through given lines, adding points at predetermined increments
        #Rho determines the minimum turning radius of bathydrone, as such, it will serve as the path stepover.
        stepover = rho 
        lastHorizontalPos = self.pointsX[0]
        for i in range(0, len(self.pointsX)-1):
            #get line slope
            currVerticalPos = self.pointsY[i]
            if self.pointsX[i+1] - self.pointsX[i] == 0:
                continue
            slope = (self.pointsY[i+1] - self.pointsY[i])/(self.pointsX[i+1] - self.pointsX[i])
            currHorizontalPos = lastHorizontalPos
            maxHorizontalPos = self.pointsX[i+1]

            #Change stepover based on path direction
            stepoverReset = 0
            if self.pointsX[i] > self.pointsX[i+1]:
                stepover = stepover * (-1)
                stepoverReset = 1

            steppingDistance = abs(currHorizontalPos - maxHorizontalPos)
            steps = 0
            
            while abs(steps * stepover) < steppingDistance:
                currVerticalPos = self.pointsY[i] + slope * (currHorizontalPos-self.pointsX[i])
                if currHorizontalPos <= maxX and currHorizontalPos >= minX:
                    if currVerticalPos <= maxY and currVerticalPos >= minY:
                        if stepover >= 0:
                           if currHorizontalPos >= self.pointsX[i] and currHorizontalPos <= self.pointsX[i+1]:
                            self.pathPointsX.append(currHorizontalPos)
                            self.pathPointsY.append(currVerticalPos)
                        else:
                            if currHorizontalPos <= self.pointsX[i] and currHorizontalPos >= self.pointsX[i+1]:
                                self.pathPointsX.append(currHorizontalPos)
                                self.pathPointsY.append(currVerticalPos)
                currHorizontalPos = currHorizontalPos + stepover
                steps = steps + 1
            lastHorizontalPos = currHorizontalPos-stepover
            if stepoverReset == 1:
                stepover = stepover * (-1)
        

        #Look ahead for rightward connections
        roundedPathX = np.around(self.pathPointsX, decimals=1)
        j = minX
        tmpFirst = -1
        tmpSecond = -1
        tmpThird = -1
        tmpFourth = -1
        topLeft = 0
        botLeft = 0
        topRight = 0
        alt = 1
        radius = stepover * 0.5
        cStep = .0625 * stepover
        while j <= maxX - 2*stepover:
            j = j + stepover
            for i in range(0, len(self.pathPointsX)):
                #Left
                if tmpFirst == -1:
                    if roundedPathX[i] == np.round(j, 1):
                        tmpFirst = i
                #Left
                if tmpSecond == -1:
                    if roundedPathX[i] == np.round(j, 1) and i != tmpFirst:
                        tmpSecond = i
                #Right
                if tmpThird == -1:
                    if roundedPathX[i] == np.round(j+stepover, 1):
                        tmpThird = i
                #Right
                if tmpFourth == -1:
                    if roundedPathX[i] == np.round(j+stepover, 1) and i != tmpThird:
                        tmpFourth = i

            #Sort to bottom left -> top left -> top right orientation.
            #Find bottom and top left
            if self.pathPointsY[tmpFirst] > self.pathPointsY[tmpSecond]:
                topLeft = tmpFirst
                botLeft = tmpSecond
            else:
                topLeft = tmpSecond
                botLeft = tmpFirst
            #Find bottom and top right
            if self.pathPointsY[tmpThird] > self.pathPointsY[tmpFourth]:
                topRight = tmpThird
                botRight = tmpFourth
            else:
                topRight = tmpFourth
                botRight = tmpThird

            #Add to path in order
            #Alternate ordering each iteration
            if alt == -1:
                if self.pathPointsY[botLeft] >= self.pathPointsY[botRight]:
                    #Draw radius based on botLeft
                    currX = self.pathPointsX[botLeft]
                    currY = self.pathPointsY[botLeft]+2*radius
                    angle = math.pi
                    for i in range(16):
                        self.pathX.append(currX+radius+radius*math.cos(angle))
                        self.pathY.append(currY+radius*math.sin(angle))
                        angle = angle + math.pi/16
                else:
                    #Draw from botRight
                    currX = self.pathPointsX[botLeft]
                    currY = self.pathPointsY[botRight]+2*radius
                    angle = math.pi
                    for i in range(16):
                        self.pathX.append(currX+radius+radius*math.cos(angle))
                        self.pathY.append(currY+radius*math.sin(angle))
                        angle = angle + math.pi/16
                #self.pathX.append(self.pathPointsX[topRight])
                #self.pathY.append(self.pathPointsY[topRight]-radius)
                alt = 1
            else:
                if self.pathPointsY[topLeft] <= self.pathPointsY[topRight]:
                    currX = self.pathPointsX[topLeft]
                    currY = self.pathPointsY[topLeft]-2*radius
                    angle = math.pi
                    for i in range(17):
                        self.pathX.append(currX+radius+radius*math.cos(angle))
                        self.pathY.append(currY+radius*math.sin(angle))
                        angle = angle - math.pi/16
                else:
                    currX = self.pathPointsX[topLeft]
                    currY = self.pathPointsY[topRight]-2*radius
                    angle = math.pi
                    for i in range(17):
                        self.pathX.append(currX+radius+radius*math.cos(angle))
                        self.pathY.append(currY+radius*math.sin(angle))
                        angle = angle - math.pi/16
                alt = -1

            tmpFirst = -1
            tmpSecond = -1
            tmpThird = -1
            tmpFourth = -1
            #Loop restarts looking for bottom left of the next index

        
                
def findBestSweepDirection(xPoints, yPoints): 
    """ Simple optimal sweep direction is parallel to the minimum width direction of the convex polygon
    This minimizes the number of turns taken by the path
    """
    #find optimal matrix direction 
    xRot = []
    yRot = []
    tempPoints = []    
    opAngleDist = max(xPoints)-min(xPoints)
    opAngle = 0
    for testAngle in range(1, 360):
        #print(opAngle)
        transformAngle = [[np.cos(np.deg2rad(testAngle)), -np.sin(np.deg2rad(testAngle))], [np.sin(np.deg2rad(testAngle)), np.cos(np.deg2rad(testAngle))]]
        tempX = [] 
        for p in range(0, len(xPoints)):
            xyMat = [[xPoints[p]], [yPoints[p]]]
            tempPoints.append(np.matmul(transformAngle, xyMat))
        for p in range(0, len(xPoints)):
            tempX.append(tempPoints[p][0])
        if max(tempX)-min(tempX) < opAngleDist:
            opAngleDist = max(tempX) - min(tempX)
            opAngle = testAngle
        tempPoints = []
    #print(opAngle) 

    #Now rotate the shape by the angle
    finalTransform = [[np.cos(np.deg2rad(opAngle)), -np.sin(np.deg2rad(opAngle))], [np.sin(np.deg2rad(opAngle)), np.cos(np.deg2rad(opAngle))]]
    tempPointsFinal = []
    for i in range(0, len(xPoints)):
         xyMatNew = [[xPoints[i]], [yPoints[i]]]
         tempPointsFinal.append(np.matmul(finalTransform, xyMatNew))
    for p in range(0, len(xPoints)):
            xRot.append(tempPointsFinal[p][0])
            yRot.append(tempPointsFinal[p][1])
    #print(xRot) 
    #print(yRot)
    
    return xRot, yRot
    
def plotGrid(xPoints, yPoints):
    plt.title("Region of interest")
    plt.xlabel("X")
    plt.ylabel("Y")

    plt.grid()
    plt.plot(xPoints, yPoints, marker='o', color='red')
    plt.axis('square')
    plt.show()

def plotPoints(xPoints, yPoints, bathyPath):
    plt.title("Waypoints Calculated")
    plt.xlabel("X")
    plt.ylabel("Y")

    plt.grid()
    myPlot = plt.plot(xPoints, yPoints, marker='o', color='red')
    plt.plot(bathyPath.pathPointsX, bathyPath.pathPointsY, marker='o', color='green', markersize='2', linestyle='none')
    plt.axis('square')
    plt.show()

def plotPath(xPoints, yPoints, bathyPath):
    plt.title("Bathydrone Boat Path")
    plt.xlabel("X")
    plt.ylabel("Y")

    plt.grid()
    myPlot = plt.plot(xPoints, yPoints, marker='o', color='red')
    #plt.plot(bathyPath.pathPointsX, bathyPath.pathPointsY, marker='o', color='green', markersize='2', linestyle='none')
    plt.plot(bathyPath.pathX, bathyPath.pathY, color='green', markersize='2')
    plt.axis('square')
    plt.show()

def main():
    #input closed area as points. 
    xPointsPoly = [0.0, 70.0, 80.0, 80.0, 70.0, 40.0, 0.0]
    yPointsPoly = [80.0, 140.0, 90.0, 30.0, 10.0, 0.0, 80.0]

    #Square
    xPointsSq = [0.0, 0.0, 100.0, 100.0, 0.0]
    yPointsSq = [0.0, 100.0, 100.0, 0.0, 0.0]

    #Irregular Octagon
    xPointsOct = [0.0, 0.0, 40.0, 80.0, 120.0, 120.0, 75.0, 30.0, 0.0]
    yPointsOct = [0.0, 60.0, 100.0,  100.0, 70.0, -10.0, -50.0, -50.0, 0.0]

    
    xRot, yRot = findBestSweepDirection(xPointsOct, yPointsOct)

    plotGrid(xRot, yRot) 
    #testGrid = plotGrid(xRot, yRot)
    boatSpecs = HardwareSpecs(1.25, 1.25) 
    bathyPath = pathPlanner(xRot, yRot, boatSpecs.rho)
    plotPoints(xRot, yRot, bathyPath)
    plotPath(xRot, yRot, bathyPath)
    
    return 0


if __name__ == "__main__":
    main()


