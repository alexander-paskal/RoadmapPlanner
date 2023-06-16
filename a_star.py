from collections import defaultdict
import numpy as np


# implementation from https://www.geeksforgeeks.org/priority-queue-in-python/#
class PQ(object):
    def __init__(self):
        self.queue = []

    def __str__(self):
        return ' '.join([str(i) for i in self.queue])

    # for checking if the queue is empty
    def isEmpty(self):
        return len(self.queue) == 0

    # for inserting an element in the queue
    def insert(self, data):
        self.queue.append(data)

    # for popping an element based on Priority
    def delete(self):
        try:
            min_val = 0
            for i in range(len(self.queue)):
                if self.queue[i][1] < self.queue[min_val][1]:
                    min_val = i
            item = self.queue[min_val]
            del self.queue[min_val]
            return item
        except IndexError:
            print()
            exit()


# requires the start and end to be added into the SPARSE and DENSE graphs
def a_star(start, goal, graph):
    # use PQ
    # treverse SPARSE GRAPH

    parent = {}
    weight = defaultdict(lambda: float('inf'))
    close = []
    openPQ = PQ()

    weight[start] = 0
    openPQ.insert((start, weight[start]))

    while not openPQ.isEmpty():
        curr, g = openPQ.delete()  # pop the lowest weight node

        close.append(curr)  # might need to be moved to end of while loop

        # go thru the children of next node
        children = graph[curr]
        for (c, w) in children:

            # if the child is goal, return the path from there
            if c == goal:
                parent[c] = curr
                print("path found:")
                sparsePT = buildPath_sparse(start, goal, parent)
                densePT = buildPath_dense(start, goal, parent, graph)
                return (sparsePT, densePT)

            # goal no found, keep checking children
            # if the weight of child is greater than the new path, change it
            EuclideanDist = np.linalg.norm(np.asarray(c) - np.asarray(goal))  # euclidean heuristic
            tempW = (g + w) + EuclideanDist
            if (weight[c] > tempW):
                weight[c] = tempW
                parent[c] = curr  # set the parent as path used

                if c not in close:
                    openPQ.insert((c, weight[c]))


def buildPath_sparse(start, goal, parent):
    path = []
    cur = goal
    while cur != start:
        path.append(cur)
        cur = parent[cur]
    path.append(start)
    return path


def buildPath_dense(start, goal, parent, graph):
    path = []
    cur = goal
    while cur != start:
        path.append(cur)
        if (cur in graph):
            # STILL WORKING
            cur = parent[cur]
        else:
            cur = parent[cur]
    path.append(start)
    return path