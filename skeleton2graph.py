import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from a_star import a_star_SPARSE

#holds each node's children 
DENSE_GRAPH = defaultdict(list)
    #(u1, v1): {set of children}

SPARSE_GRAPH = {}
PATH_DIR = {}
intersections = []

#dense --> intersection points & return SPARSEGRAPH with intersections included
def extract_intersections(dense_graph):
    # intersections = []
    for node, children in dense_graph.items():
        if (len(children) > 2) or (len(children)==1):
            intersections.append(node)
    return set(intersections)

def connectSPARSE():
    for inter in intersections: # for each intersection, find the nearby ones
        
        visited = []         
        i = 0 #weight
        parent = inter
        st = [] #stack to track the frontier
        st.append((inter, i, parent)) # add the current node (intersection)
        
        #iterate thru each of the currIntersection's children, until the next intersection is found
        while len(st)>0:
            (curr,i, parent) = st.pop() # pick the next child
            visited.append(curr) # mark the node that was popped as visited
            
            #an intersection not the current one
            if((curr != inter) and (curr in intersections)): 
                
                #TODO: find way to track back to iteration=1 children and add them as connectors

                #ADD TO SPARSE USING MAPING DICT TO SHOW PATHS
                if inter in SPARSE_GRAPH: 
                    SPARSE_GRAPH[inter].append((curr, i))
                else:
                    SPARSE_GRAPH[inter] = [(curr, i)]

                #connect the direction
                if parent == inter:
                    #if intersections are direct neighbors, then the node is next intersection
                    PATH_DIR[(inter, curr)] = curr 
                else:
                    #if there is connector nodes on the way to the nxt intersection, save the first connector
                    PATH_DIR[(inter, curr)] = parent

            else : #is just a connector node & can be added onto the graph
                
                for c in DENSE_GRAPH[curr]:
                    #parent is the connect node it came from
                    if i==0: 
                        parent = curr
                    else: 
                        parent = parent

                    if not(c in visited):
                        pathcost = np.linalg.norm(np.asarray(c) - np.asarray(curr))                     
                        st.append((c, i+pathcost, parent)) #add the children with 1 level more {can use level 1 children as connectors}

        # connectors = DENSE_GRAPH[inter]

#implementation from https://www.geeksforgeeks.org/priority-queue-in-python/#
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
   

#plotting
def plot_dense_graph():
    for (u, v) in DENSE_GRAPH:
        plt.scatter(u, v, c="red", s=6)

    for (u, v), children in DENSE_GRAPH.items():
        for (u1, v1) in children:
            plt.plot([u, u1], [v, v1], c="blue", alpha=0.5)
    # plt.show()

def plot_graph(graph):
    for (u, v) in graph:
        plt.scatter(u, v, c="purple", s=6)

    for (u, v), values in graph.items():
        # (curr, i) = v in values
        # children, weights = v
        for ((u1, v1), w) in values:
            # print(u, u1, v, v1)
            plt.plot([u, u1], [v, v1], c="black", alpha=0.5)

def main():

    arr = np.load("skeleton.npy")

    H, W = arr.shape

    xx, yy = np.meshgrid(
            np.arange(H),
            np.arange(W)
    )

    #for each pixel nx2 
    inds = np.vstack([xx.flatten(), yy.flatten()]).T

    #how many of all pixels are points(skeleton) in the graph
    mask = arr.T.flatten()

    dense_nodes = inds[mask]


    for node in dense_nodes:
        u, v = node

        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                if i == 0 and j == 0:
                    continue

                u_child = u + i
                v_child = v + j

                #iterate thru every neighbor
                if arr[u_child, v_child] == 1:
                    DENSE_GRAPH[(u, v)].append((u_child, v_child))

    # plot_dense_graph()

    # ints = extract_intersections(DENSE_GRAPH)
    # plt.scatter(*np.array(ints).T, c="green", s=30)
    # print(len(ints))
    # plt.show()

    extract_intersections(DENSE_GRAPH)
    connectSPARSE()
    # print(SPARSE_GRAPH)


    start = (35, 210)
    goal = (147, 3)
    (sparsePT, densePT) = a_star_SPARSE(start, goal, SPARSE_GRAPH, DENSE_GRAPH)
    print("sparse: " + str(sparsePT))
    print("dense: "+ str(densePT))

    plot_graph(SPARSE_GRAPH)
    plt.show()


if __name__ == '__main__':
    main()