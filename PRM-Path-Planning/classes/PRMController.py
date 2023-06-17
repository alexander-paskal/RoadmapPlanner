from collections import defaultdict
import sys
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle
import numpy as np
from sklearn.neighbors import NearestNeighbors
import shapely.geometry
import argparse

from .Dijkstra import Graph, dijkstra, to_array
from .Utils import Utils
import time


class PRMController:
    def __init__(self, numOfRandomCoordinates, env, current, destination):
        
        self.numOfCoords = numOfRandomCoordinates
        self.coordsList = np.array([])
        self.env = env
        self.current = np.array(current)
        self.destination = np.array(destination)
        self.graph = Graph()
        self.utils = Utils()
        self.solutionFound = False
        if self.checkPointCollision(np.array(current)) or self.checkPointCollision(np.array(destination)):
            print("illegal starting or goal pos")
            assert 1==0

    def runPRM(self, initialRandomSeed, saveImage=True):
        seed = initialRandomSeed
        # Keep resampling if no solution found
        start_g = time.time()
        while(not self.solutionFound):
            print("Trying with random seed {}".format(seed))
            np.random.seed(seed)

            # Generate n random samples called milestones
            
            self.genCoords()

            # Check if milestones are collision free
            self.checkIfCollisonFree()

            # Link each milestone to k nearest neighbours.
            # Retain collision free links as local paths.
            self.findNearestNeighbour()
            end_g = time.time()
            

            # Search for shortest path from start to end node - Using Dijksta's shortest path alg
            start = time.time()
            self.shortestPath()
            end = time.time()

            seed = np.random.randint(1, 100000)
            self.coordsList = np.array([])
            self.graph = Graph()
        print("time elapsed g: ", end_g - start_g)
        print("time elapsed: ", end - start)
        if(saveImage):
            plt.savefig("{}_samples.png".format(self.numOfCoords))
        plt.show()

    def genCoords(self):
        maxSize_x = self.env.shape[0]
        maxSize_y = self.env.shape[1]

        x = np.random.randint(0, maxSize_x, size=(self.numOfCoords, 1))
        y = np.random.randint(0, maxSize_y, size=(self.numOfCoords, 1))
        self.coordsList = np.concatenate((x,y), axis=-1)
        #print(self.coordsList.shape)
        #assert 1==0

        # Adding begin and end points
        self.current = self.current.reshape(1, 2)
        self.destination = self.destination.reshape(1, 2)
        self.coordsList = np.concatenate(
            (self.coordsList, self.current, self.destination), axis=0)

    def checkIfCollisonFree(self):
        collision = False
        self.collisionFreePoints = np.array([])
        for point in self.coordsList:
            collision = self.env[point[0], point[1]]
            if(collision == 1):
                if(self.collisionFreePoints.size == 0):
                    self.collisionFreePoints = point
                else:
                    self.collisionFreePoints = np.vstack(
                        [self.collisionFreePoints, point])
        self.plotPoints(self.collisionFreePoints)

    def findNearestNeighbour(self, k=5):
        X = self.collisionFreePoints
        knn = NearestNeighbors(n_neighbors=k)
        knn.fit(X)
        distances, indices = knn.kneighbors(X)
        self.collisionFreePaths = np.empty((1, 2), int)

        for i, p in enumerate(X):
            # Ignoring nearest neighbour - nearest neighbour is the point itself
            for j, neighbour in enumerate(X[indices[i][1:]]):
                start_line = p
                end_line = neighbour
                if(not self.checkPointCollision(start_line) and not self.checkPointCollision(end_line)):
                    if(not self.checkLineCollision(start_line, end_line)):
                        self.collisionFreePaths = np.concatenate(
                            (self.collisionFreePaths, p.reshape(1, 2), neighbour.reshape(1, 2)), axis=0)

                        a = str(self.findNodeIndex(p))
                        b = str(self.findNodeIndex(neighbour))
                        self.graph.add_node(a)
                        self.graph.add_edge(a, b, distances[i, j+1])
                        x = [p[0], neighbour[0]]
                        y = [p[1], neighbour[1]]
                        plt.plot(y, x)

    def shortestPath(self):
        self.startNode = str(self.findNodeIndex(self.current))
        self.endNode = str(self.findNodeIndex(self.destination))

        dist, prev = dijkstra(self.graph, self.startNode)

        pathToEnd = to_array(prev, self.endNode)

        if(len(pathToEnd) > 1):
            self.solutionFound = True
        else:
            return

        # Plotting shorest path
        pointsToDisplay = [(self.findPointsFromNode(path))
                           for path in pathToEnd]
        #print(pointsToDisplay)
        x = [int(item[0]) for item in pointsToDisplay]
        y = [int(item[1]) for item in pointsToDisplay]
        plt.plot(y, x, c="blue", linewidth=3.5)

        pointsToEnd = [str(self.findPointsFromNode(path))
                       for path in pathToEnd]
        pointsToEnd_output = [self.findPointsFromNode(path)
                       for path in pathToEnd]
        pointsToEnd_output = np.array(pointsToEnd_output)
        np.save("prm_path_narrow.npy", pointsToEnd_output)
        print("****Output****")

        print("The quickest path from {} to {} is: \n {} \n with a distance of {}".format(
            self.collisionFreePoints[int(self.startNode)],
            self.collisionFreePoints[int(self.endNode)],
            " \n ".join(pointsToEnd),
            str(dist[self.endNode])
        )
        )

    def checkLineCollision(self, start_line, end_line):
        collision = False
        n = max(abs(start_line[0] - end_line[0]), abs(start_line[1] - end_line[1]))
        if n <= 1: return False
        n += 1
        x = np.linspace(start_line[0], end_line[0], n)[1:-1].reshape([-1,1]).round().astype(int)
        y = np.linspace(start_line[1], end_line[1], n)[1:-1].reshape([-1,1]).round().astype(int)
        line_points = np.concatenate((x,y),axis=-1)
        for p in range(line_points.shape[0]):
            if self.checkPointCollision(line_points[p,:]): return True
        return False

    def findNodeIndex(self, p):
        return np.where((self.collisionFreePoints == p).all(axis=1))[0][0]

    def findPointsFromNode(self, n):
        return self.collisionFreePoints[int(n)]

    def plotPoints(self, points):
        x = [item[0] for item in points]
        y = [item[1] for item in points]
        plt.scatter(y, x, c="black", s=1)

    def checkPointCollision(self, point):

        if self.env[point[0],point[1]] == 0:
                return True
        return False
