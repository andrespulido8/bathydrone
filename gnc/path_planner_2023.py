# LAST UPDATE: Nicholas Sardinia, Febuary 20 2023
"""
Updated algorithm to generate waypoints for convex polygons (semi-functional)
"""

#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

# Define the boat hardware
class HardwareSpecs:
    # accounts for radius of curvature and side-scan sonar geometry
    def __init__(self, D, alpha_2):
        self.rho = D*np.tan(alpha_2)
        # rho - radius that constrains the spacing between each
        # D - estimated depth of water
        # alpha_2 = Measured angle related to coverage of side-scan sonar

"""
Generates path, starting from lower-left corner. 
# Lawnmower lines for full convex coveragae.
# Returns path as a series of grid points from bottom left to top right
"""
class pathPlanner:
    def __init__(self, pointsX, pointsY, rho):
        self.pointsX = pointsX
        self.pointsY = pointsY
        self.rho = rho
        self.pathPointsX = []
        self.pathPointsY = []
        self.pathX = []
        self.pathY = []


        pathDirection = 0
        UP = 1
        DOWN = -1
        
        
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
                        self.pathPointsX.append(currHorizontalPos)
                        self.pathPointsY.append(currVerticalPos)
                currHorizontalPos = currHorizontalPos + stepover
                steps = steps + 1
            lastHorizontalPos = currHorizontalPos-stepover
            if stepoverReset == 1:
                stepover = stepover * (-1)
        
            

        #Now, draw lines between any two points at the same horizontal position.
        #Begin from minimum x
        roundedPathX = np.around(self.pathPointsX, decimals=1)
        j = 0
        while j < maxX:
            j = j + stepover
            for i in range(0, len(self.pathPointsX)):
                if roundedPathX[i] == np.round(j, 1):
                    self.pathX.append(self.pathPointsX[i])
                    self.pathY.append(self.pathPointsY[i])
        
                    
        
    
                    
        
            



#path_planner improvements
class pathPlannerMarch:
    def __init__(self, pointsX, pointsY, rho):
        self.pointsX = pointsX
        self.pointsY = pointsY
        self.rho = rho
        self.pathX = []
        self.pathY = []

        pathDirection = 0
        UP = 1
        DOWN = -1

        minXDex = 0
        minX = pointsX[0]
        for i in range(0, len(pointsX)):
            if pointsX[i] < minX:
                minX = pointsX[i]
                minXDex = i

        self.pathX.append(pointsX[minXDex])
        self.pathY.append(pointsY[minXDex])

        #First "Iteration Righward" = pointsX[minXDex] + rho
        firstIter = pointsX[minXDex] + rho

        xLeftBot = 0
        xRightBot = 0
        xLeftTop = 0
        xRightTop = 0

        #Vertical line
        if (pointsX[0] == pointsX[1]):
            #there is a vertical line
            interpTop = pointsY[1]
        else:
            for i in range(0, len(pointsX)):
                if pointsY[i] >= self.pathY[0] and pointsX[i] <= firstIter:
                    xLeftTop = i
                if pointsY[i] >= self.pathY[0] and pointsX[i] >= firstIter:
                    xRightTop = i
            interpTop = self.pathY[0] + (pointsY[xRightTop]-pointsY[xLeftTop])/(abs(pointsX[xLeftTop]-pointsX[xRightTop]))*(rho)
        #X-2, because in a closed polygon, the first point will always equal the last point
        if (pointsX[0] == pointsX[len(pointsX)-2]):
            #there is a vertical line below our point
            interpBot = pointsY[len(pointsX)-2]
        else:
            for i in range(0, len(pointsX)):
                if pointsY[i] <= self.pathY[0] and pointsX[i] <= firstIter:
                    xLeftBot = i
                if pointsY[i] <= self.pathY[0] and pointsX[i] >= firstIter:
                        xRightBot = i
                interpBot = self.pathY[0] - (pointsY[xRightBot]-pointsY[xLeftBot])/(pointsX[xLeftBot]-pointsX[xRightBot])*(rho)
        
        #Choose path direction and place first waypoint
        if (self.pathY[0] - interpBot >= interpTop-self.pathY[0]):
            pathDirection = -1
            self.pathX.append(firstIter)
            self.pathY.append(interpBot)
        else:
            pathDirection = 1
            self.pathX.append(firstIter)
            self.pathY.append(interpTop)

        #Sweep the shape

        #find max point in x(stopping criterion)
        maxX = pointsX[0]
        for i in range(0, len(pointsX)):
            if pointsX[i] > maxX:
                maxX = pointsX[i]

        currDex = 0
        currX = self.pathX[0]
        currY = self.pathY[0]

        #Generates Waypoints
        while True:
            currDex = currDex + 1
            if maxX < pointsX[currDex] + rho:
                break
            
                
#Simple optimal sweep direction is parallel to the minimum width direction of the convex polygon
#This minimizes the number of turns taken by the path
def findBestSweepDirection(xPoints, yPoints): 
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
    plt.plot(bathyPath.pathX, bathyPath.pathY, marker='o', color='green', markersize='2')
    plt.axis('square')
    plt.show()

def main():
    #input closed area as points. 
    xPointsPoly = [0.0, 0.0, 100.0, 90, 0.0]
    yPointsPoly = [0.0, 100.0, 100.0, 0.0, 0.0]

    #Square
    xPointsSq = [0.0, 0.0, 100.0, 100.0, 0.0]
    yPointsSq = [0.0, 100.0, 100.0, 0.0, 0.0]

    #Irregular Octagon
    xPointsOct = [0.0, 0.0, 40.0, 80.0, 120.0, 120.0, 75.0, 30.0, 0.0]
    yPointsOct = [0.0, 60.0, 100.0,  100.0, 70.0, -10.0, -50.0, -50.0, 0.0]

    
    xRot, yRot = xPointsOct, yPointsOct

    plotGrid(xRot, yRot) 
    #testGrid = plotGrid(xRot, yRot)
    boatSpecs = HardwareSpecs(1.25, 1.25) 
    bathyPath = pathPlanner(xRot, yRot, boatSpecs.rho)
    plotPoints(xRot, yRot, bathyPath)
    plotPath(xRot, yRot, bathyPath)
    
    

    return 0


if __name__ == "__main__":
    main()


