#!/usr/bin/env python3
""" Generate a complete coverage path for a side-scan sonar vessel on an arbitrary convex region.
    Author: Nicholas Sardinia
"""
import math
import os
import sys
from typing import List, Tuple

import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from py2opt.routefinder import RouteFinder
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
from shapely.prepared import prep


class PathPlanner:
    """
    The bathydrone path planner.

    Attributes:
        self.bounding: BoundingRegion object to be pathed.
        self.sideScanAngle: angle of the side-scan sonar. (NOT implemented yet)
        self.maxRange: maximum range of the side-scan sonar.
        self.minRange: minimum range of the side-scan sonar.
        self.noRequireParallel: disable parallel pathing requirement.
    """

    def __init__(self, sideScanAngle=30, **system_params):
        self.bounding = None
        self.sideScanAngle = sideScanAngle
        self.maxRange = system_params.get("maxRange", None)
        self.minRange = system_params.get("minRange", None)
        self.maxDepth = system_params.get("maxDepth", None)
        self.noRequireParallel = system_params.get("noRequireParallel", None)

    def path_region(self, path_dist_ratio=0.05, tol_factor=1, is_plot=True):
        """Generate a complete coverage path for a side-scan sonar vessel on an arbitrary convex region."""
        # GENERATE POLYGON OBJECT (required for shapely)
        geom = Polygon(self.bounding.polygonVertices)
        xList, yList = list(zip(*self.bounding.polygonVertices))

        # APPROXIMATE DECOMPOSITION
        # Path distance approximation (when physical parameters are unknown)
        pathDist = abs((max(xList) - min(xList)) * path_dist_ratio)
        # Decomposition Tolerance Constant (SL-Concavity)
        tol = pathDist * tol_factor
        pTest = self.decomposePolygon(tol, geom)

        # GENERATE PATH FOR EACH DECOMPOSED POLYGON
        startPos = [min(xList), min(yList)]
        pathCenters, testPathArr = self.generatePathsOfDecomposedRegions(
            pTest, pathDist, [startPos]
        )

        # APPEND PATHS FOR EACH DECOMPOSED POLYGON INTO ONE OPTIMIZED PATH
        optimized_path = self.appending_order_of_paths(testPathArr)

        # 2-OPT HEURISTIC
        best_distance, best_route = self.orderTwoOpt(pathCenters)  # understand why pathCenters is getting fed into 2 opt 

        # PLOTTING
        if is_plot:
            self.plotPath(
                xList, yList, testPathArr, pathCenters, best_route, best_distance
            )
        else:
            return testPathArr

    def get_bounding_polygon(self, **boundingInput):
        """Create a Bounding Polygon Object to Store in the PathPlanner."""
        # TODO add support for depth map
        if "csvName" in boundingInput:
            # fetch from CSV
            setattr(
                self,
                "bounding",
                BoundingRegion(bounding_polygon=None, csvName=boundingInput["csvName"]),
            )
        elif "bounding_polygon" in boundingInput:
            # get from bounding polygon provided directly as an array of tuples.
            setattr(
                self,
                "bounding",
                BoundingRegion(
                    bounding_polygon=boundingInput["bounding_polygon"], csvName=None
                ),
            )

    def generate_path(
        self, polygon_points: List[Tuple[float, float]], path_dist: float
    ) -> Tuple[List[List[float]], float, bool, List[Polygon], Polygon]:
        """
        Generate an optimized path through the polygon.

        Args:
            polygon_points (List[Tuple[float, float]]): List of vertices of the polygon.
            path_dist (float): Distance between waypoints.

        Returns:
            Tuple[List[List[float]], float, bool, List[Polygon], Polygon]:
                - chosenPath: List of path points
                - bestPL: Best path length
                - is_empty: Flag indicating if the path is empty
                - bestGrid: Partition grid used for path generation
                - bestGeom: Polygon geometry used for path generation
        """
        TURN_WEIGHT = 0.4  # Weight for number of turns in optimization
        best_path_length = float("inf")
        best_path = []
        best_grid = None
        best_geom = None
        is_empty = True

        # Test different angles for optimal path orientation
        for angle in range(0, 180, 30):
            # Rotate polygon
            transform = [
                [np.cos(np.deg2rad(angle)), -np.sin(np.deg2rad(angle))],
                [np.sin(np.deg2rad(angle)), np.cos(np.deg2rad(angle))],
            ]
            rotated_points = [
                list(np.dot(transform, point)) for point in polygon_points
            ]

            # Generate grid and path
            geom = Polygon(rotated_points)
            grid = self.partition(geom, path_dist)

            if len(grid) < 2:
                continue

            path, path_length, num_turns = PathPlanner._generate_path_for_grid(grid)

            if not path:
                continue

            # Rotate path back
            inv_transform = [
                [np.cos(np.deg2rad(-angle)), -np.sin(np.deg2rad(-angle))],
                [np.sin(np.deg2rad(-angle)), np.cos(np.deg2rad(-angle))],
            ]
            path = [list(np.dot(inv_transform, point)) for point in path]

            # Calculate cost considering path length and turns
            cost = 0.001 * path_length + TURN_WEIGHT * num_turns

            if cost < best_path_length:
                best_path = path
                best_path_length = path_length
                best_grid = grid
                best_geom = geom
                is_empty = False

        return best_path, best_path_length, is_empty, best_grid, best_geom

    @staticmethod
    def _generate_path_for_grid(
        grid: List[Polygon],
    ) -> Tuple[List[List[float]], float, int]:
        """
        Generate a path through the grid cells.

        Args:
            grid (List[Polygon]): List of grid cells.

        Returns:
            Tuple[List[List[float]], float, int]:
                - path: List of path points
                - path_length: Total length of the path
                - num_turns: Number of turns in the path
        """
        path = []
        num_turns = 0
        direction = 1  # 1 = up, -1 = down

        # Get grid boundaries
        x_coords = [cell.bounds[0] for cell in grid]
        min_x = min(x_coords)
        max_x = max(x_coords)
        cell_width = abs(grid[0].bounds[2] - grid[0].bounds[0])

        curr_x = min_x
        while curr_x <= max_x:
            # Get cells in current slice
            slice_cells = [
                cell for cell in grid if abs(cell.bounds[0] - curr_x) < 1e-10
            ]

            if slice_cells:
                num_turns += 1
                path.extend(PathPlanner._get_slice_path(slice_cells, direction))
                direction *= -1

            curr_x += cell_width

        # Calculate path length
        path_length = PathPlanner._calculate_path_length(path)

        return path, path_length, num_turns

    @staticmethod
    def _get_slice_path(cells: List[Polygon], direction: int) -> List[List[float]]:
        """
        Generate path points for a vertical slice of cells.

        Args:
            cells (List[Polygon]): List of grid cells in the slice.
            direction (int): Direction of the path (1 = up, -1 = down).

        Returns:
            List[List[float]]: List of path points for the slice.
        """
        if direction == 1:
            start_cell = cells[0]
            end_cell = cells[-1]
        else:
            start_cell = cells[-1]
            end_cell = cells[0]

        return [
            [
                (start_cell.bounds[0] + start_cell.bounds[2]) / 2,
                (start_cell.bounds[1] + start_cell.bounds[3]) / 2,
            ],
            [
                (end_cell.bounds[0] + end_cell.bounds[2]) / 2,
                (end_cell.bounds[1] + end_cell.bounds[3]) / 2,
            ],
        ]

    @staticmethod
    def _calculate_path_length(path: List[List[float]]) -> float:
        """
        Calculate the total length of the path.

        Args:
            path (List[List[float]]): List of path points.

        Returns:
            float: Total length of the path.
        """
        total_length = 0
        for i in range(len(path) - 1):
            dist_x = (path[i][0] - path[i + 1][0]) * 69
            dist_y = (path[i][1] - path[i + 1][1]) * 54.6
            total_length += math.sqrt(dist_x**2 + dist_y**2)
        return total_length

    @staticmethod
    def grid_bounds(geom, delta):
        """
        Input: geom (shapely polygon), delta (distance between path lines)
        Output: grid boundaries from which to draw path.
        """

        minx, miny, maxx, maxy = geom.bounds
        nx = int((maxx - minx) / delta)
        ny = int((maxy - miny) / delta)
        gx, gy = np.linspace(minx, maxx, nx), np.linspace(miny, maxy, ny)

        grid = []
        for i in range(len(gx) - 1):
            for j in range(len(gy) - 1):
                poly_ij = Polygon(
                    [
                        [gx[i], gy[j]],
                        [gx[i], gy[j + 1]],
                        [gx[i + 1], gy[j + 1]],
                        [gx[i + 1], gy[j]],
                    ]
                )
                grid.append(poly_ij)
        return grid

    def partition(self, geom, delta):
        """Define a grid of cells for a polygon."""
        prepared_geom = prep(geom)
        grid = list(filter(prepared_geom.covers, self.grid_bounds(geom, delta)))
        return grid

    @staticmethod
    def listConvexHull(polyPoints):
        """Find the convex hull of a polygon."""
        cHull = ConvexHull(polyPoints)
        listHull = []
        for i in range(len(cHull.vertices)):
            listHull.append(polyPoints[cHull.vertices[i]])
        return listHull

    @staticmethod
    def concavity_checker(polyPoints, polyGeom):
        """Take a polygon, and export an inorder list of SL-concavities"""

        conHullC = polyGeom.convex_hull.get_coordinates()
        conHull = list(conHullC.itertuples(index=False, name=None))

        concavities = [0] * len(polyPoints)
        for i in range(len(polyPoints)):
            if polyPoints[i] in conHull:
                concavities[i] = 0
            else:
                # find bounding hull points
                if i == 0:
                    j = len(polyPoints) - 1
                else:
                    j = i

                while j >= 0:
                    j = j - 1
                    if polyPoints[j] in conHull:
                        b1 = polyPoints[j]
                        break
                j = i
                if i == (len(polyPoints) - 1):
                    j = 0
                while j < len(polyPoints) - 1:
                    j = j + 1
                    if polyPoints[j] in conHull:
                        b2 = polyPoints[j]
                        break
                # Find perpendicular distance from line b1->b2 to point polyPoints[i]
                concavities[i] = abs(
                    (b1[0] - b2[0]) * (b1[1] - polyPoints[i][1])
                    - (b1[1] - b2[1]) * (b1[0] - polyPoints[i][0])
                ) / math.sqrt(math.pow(b1[0] - b2[0], 2) + math.pow(b1[1] - b2[1], 2))
        return concavities

    @staticmethod
    def resolveConvex(polyPoints, rIndex, concavities):
        """Resolve a convex polygon into two polygons."""
        # TODO improve optimization

        # tunable constants for heuristic
        ignoreRange = [rIndex + len(concavities) / 5.0, rIndex - len(concavities) / 5.0]
        sc = 0.1
        sd = 1
        # Find split points
        bestScore = 0
        bestDex = 0
        for i in range(len(polyPoints)):
            # Check unless in ignore range
            if (i >= ignoreRange[0] or i <= ignoreRange[1]) and abs(
                sd * math.dist(polyPoints[rIndex], polyPoints[i])
            ) != 0.0:
                score = (1 + sc * concavities[i]) / (
                    abs(sd * math.dist(polyPoints[rIndex], polyPoints[i]))
                )
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

    def makeConvexRec(self, polyPoints, tolerance, polyStore, polyPointsGeom):
        """Take a polygon and apply approximate convex decomposition.

        Source: Lien et.al. "Approximate Convex Decomposition of Polygons"
        https://www.sciencedirect.com/science/article/pii/S0925772105001008
        """
        concavityList = self.concavity_checker(polyPoints, polyPointsGeom)
        t = max(concavityList)
        if t <= tolerance:
            if polyPoints not in polyStore:
                polyStore.append(polyPoints)
            return polyPoints
        else:
            # Don't do this more than once per recursion
            ps1, ps2, polyGeom1, polyGeom2 = self.resolveConvex(
                polyPoints, concavityList.index(max(concavityList)), concavityList
            )
            self.makeConvexRec(ps1, tolerance, polyStore, polyGeom1)
            self.makeConvexRec(ps2, tolerance, polyStore, polyGeom2)

    def makeConvex(self, polyPoints, tolerance, polyPointsGeom):
        """apply approximate convex decompsition to a simple polygon"""
        ps = []
        self.makeConvexRec(polyPoints, tolerance, ps, polyPointsGeom)
        return ps

    @staticmethod
    def get_polygon_from_csv_external(csvName):
        """get region of interest from csv (edge detection)"""
        xList = []
        yList = []
        file_path = os.path.join(os.getcwd(), csvName)
        df = pd.read_csv(file_path, usecols=["Latitude", "Longitude"])
        yList = df.Latitude.values.tolist()
        xList = df.Longitude.values.tolist()

        return xList, yList

    def generatePathsOfDecomposedRegions(self, pTest, pathDistOrig, startPoint):
        """generate a coverage path of a set of approximately convex polygons"""
        testPathArr = []
        pathCenters = startPoint
        variablePathDist = pathDistOrig
        for i in range(len(pTest)):
            x1, y1 = zip(*pTest[i])
            maxArea = (max(x1) - min(x1)) * (max(y1) - min(y1))
            if maxArea > (5 * pathDistOrig * pathDistOrig):
                genPath = self.generate_path(pTest[i], variablePathDist)
                if len(genPath[0]) > 1:
                    testPathArr.append(genPath[0])
                    xCenter, yCenter = zip(*genPath[0])
                    xLoc = (max(xCenter) + min(xCenter)) / 2
                    yLoc = (max(yCenter) + min(yCenter)) / 2
                    pathCenters.append([xLoc, yLoc])

        return pathCenters, testPathArr

    def decomposePolygon(self, tolerance, polygonToDecompose):
        """Decompose polygon in to approximately convex sub-polygons

        Args:
            tolerance (float): tolerance for decomposition
            polygonToDecompose (Polygon): polygon to decompose
        """
        poly = gpd.GeoSeries([polygonToDecompose])
        polyCoords = poly.get_coordinates()
        polyCoordsList = list(polyCoords.itertuples(index=False, name=None))
        decomposedPolygon = self.makeConvex(polyCoordsList, tolerance, poly)
        return decomposedPolygon

    @staticmethod
    def orderTwoOpt(pathCenters):
        """approximate convex decomposition tsp solution with 2-opt algorithm"""
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
        for i in range(len(pathCenters) - 1):
            initDist += distMat[i][i + 1]

        # 2-OPT ITERATION (from py2opt library, stable)

        # stops route_finder from printing timer
        sys.stdout = open(os.devnull, "w")

        route_finder = RouteFinder(distMat, pathLocs, iterations=5)
        best_distance, best_route = route_finder.solve()

        # re-enables command-line prints
        sys.stdout = sys.__stdout__

        return best_distance, best_route

    @staticmethod
    def plotPath(xList, yList, testPathArr, pathCenters, best_route, best_distance):
        """generate matplotlib plot of the competed path"""
        # PLOTTING SETUP
        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        ax1.axes.get_xaxis().set_visible(False)
        ax1.axes.get_yaxis().set_visible(False)

        # PLOTTING PATHS
        # print(testPathArr)
        ax1.plot(xList, yList, c="red", marker="o", markersize="0.25")
        for i in range(len(testPathArr)):
            x1, y1 = zip(*testPathArr[i])
            ax1.plot(x1, y1, c="green", marker="o", markersize="0.5", zorder=1)

        # PLOTTING VISIT ORDER
        j = 0
        xc2, yc2 = zip(*pathCenters)

        for xIm, yIm in zip(xc2, yc2):
            ax1.text(
                xIm, yIm, str(j), color="grey", fontsize=12, fontweight="bold", zorder=3
            )
            # plt.scatter(xIm, yIm, s=sSize, facecolors='white', edgecolors='grey', zorder=2)
            j += 1

        # SHOW PLOT
        plt.axis("equal")
        plt.show()

    @staticmethod
    def plotPath3D(xList, yList, testPathArr, pathCenters):
        """Generate 3D plot of the path and depthmap."""

        # TODO add depth map
        # PLOTTING SETUP
        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        ax1.axes.get_xaxis().set_visible(False)
        ax1.axes.get_yaxis().set_visible(False)

        # 3d plotting
        ax = plt.axes(projection="3d")

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
        max_radius = (max(xList) - min(xList)) / 2.25
        depths = np.linspace(0, max_depth, num_rings)
        ring_radius = np.linspace(0, max_radius, num_rings)

        for depth, radius in zip(depths, ring_radius):
            theta = np.linspace(0, 2 * np.pi, 100)
            x = avg_center[0] + radius * np.cos(theta)
            y = avg_center[1] + radius * np.sin(theta)
            z = np.full_like(x, depth)
            ax.plot3D(x, y, z, color="blue")

        for i in range(len(testPathArr)):
            x1, y1 = zip(*testPathArr[i])
            ax.plot3D(
                x1, y1, lakeZero, c="black", marker="o", markersize="0.5", zorder=1
            )

        plt.show()
    
    @staticmethod
    def appending_order_of_paths(testPathArr):
        if not testPathArr:  # If empty array
            return []
        if len(testPathArr) == 1:  # If only one path
            return testPathArr[0]
            
        # Start with the first path
        optimized_path = list(testPathArr[0])
        
        # Iteratively add remaining paths
        for next_path in testPathArr[1:]:
            if not next_path:  # Skip empty paths
                continue
                
            min_distance = float('inf')
            best_configuration = None
            
            # Try all possible connections between current path and next path
            candidates = [
                (optimized_path[-1], next_path[0], False, False),   # End to start
                (optimized_path[-1], next_path[-1], False, True),   # End to end
                (optimized_path[0], next_path[0], True, False),     # Start to start
                (optimized_path[0], next_path[-1], True, True)      # Start to end
            ]
            
            # Find the best connection configuration
            for p1, p2, reverse_current, reverse_next in candidates:
                dist = math.dist(p1, p2)
                if dist < min_distance:
                    min_distance = dist
                    best_configuration = (reverse_current, reverse_next)
            
            # Apply the best configuration
            if best_configuration:
                reverse_current, reverse_next = best_configuration
                if reverse_current:
                    optimized_path.reverse()
                next_path_copy = list(next_path)
                if reverse_next:
                    next_path_copy.reverse()
                    
                # Connect the paths
                optimized_path.extend(next_path_copy)
        
        return optimized_path
        
# TODO add manipulations to bounding region polygon.
class BoundingRegion:
    """
    The bounding polygon for the region of interest.

    Attributes:
        polygonVertices (list of tuples): vertices forming exterior of the bounding polygon.
    """

    def __init__(self, bounding_polygon, csvName):
        if csvName is None:
            assert bounding_polygon is not None, "No polygon provided."
            self.polygonVertices = [tuple(xy_list) for xy_list in bounding_polygon]
        else:
            assert (
                bounding_polygon is None
            ), "Provide either bounding polygon or csv. Set other as None."
            self.polygonVertices = self.get_polygon_from_csv(csvName)

    @staticmethod
    def get_polygon_from_csv(csvName):
        """get region of interest from csv (manual)"""
        file_path = os.path.join(os.getcwd(), "csv", csvName)
        df = pd.read_csv(file_path, usecols=["Latitude", "Longitude"])
        return list(zip(df.Latitude.values.tolist(), df.Longitude.values.tolist()))


def main():
    """
    Example usage of the PathPlanner class.

    1. Create a PathPlanner object.
    2. Set the bounding polygon from csv.
    3. Generate and display a coverage path for the region.
    """
    my_planner = PathPlanner()
    my_planner.get_bounding_polygon(csvName="newnans-lake-coords.csv")
    my_planner.path_region()
    return 0


if __name__ == "__main__":
    main()
