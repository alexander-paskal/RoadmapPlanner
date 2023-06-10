import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

#checking for each intersection scenario
MASKS = np.array([
    [[1, 0, 1],
     [0, 1, 0],
     [0, 1, 0]],
    [[0, 1, 0],
     [0, 1, 1],
     [1, 0, 0]],
    [[0, 0, 1],
     [1, 1, 0],
     [0, 0, 1]],
    [[1, 0, 0],
     [0, 1, 1],
     [0, 1, 0]],

    [[0, 1, 0],
     [0, 1, 0],
     [1, 0, 1]],
    [[0, 0, 1],
     [1, 1, 0],
     [0, 1, 0]],
    [[1, 0, 0],
     [0, 1, 1],
     [1, 0, 0]],
    [[0, 1, 0],
     [1, 1, 0],
     [0, 0, 1]],

    [[1, 0, 0],
     [0, 1, 0],
     [1, 0, 1]],
    [[1, 0, 1],
     [0, 1, 0],
     [1, 0, 0]],
    [[1, 0, 1],
     [0, 1, 0],
     [0, 0, 1]],
    [[0, 0, 1],
     [0, 1, 0],
     [1, 0, 1]],

])


#holds each node's children 
DENSE_GRAPH = defaultdict(list)
    #(u1, v1): {set of children}

SPARSE_GRAPH = {}

#dense --> intersection points & return SPARSEGRAPH with intersections included
def extract_intersections(dense_graph):
    intersections = []
    for node, children in dense_graph.items():
        if len(children) > 2:
            intersections.append(node)

            #add in the intersections with the children into the sparse graph

            #find the path the connectors lead to

            # SPARSE_GRAPH[node] = children #TODO: Add in the path ID

    return set(intersections)

def connectSPARSE():
    

    intersections = extract_intersections(DENSE_GRAPH)

    for inter in intersections: # for each intersection, find the nearby ones
        
        visited = [] 
        
        print(inter)
        
        i = 0
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

                #ADD TO SPARSE WITH PARENT 
                if parent == inter: # parent is the current intersection (no intermediate nodes)
                    if inter in SPARSE_GRAPH:
                        SPARSE_GRAPH[inter].append((curr, i))
                    else:
                        SPARSE_GRAPH[inter] = [(curr, i)]
                else:
                    if inter in SPARSE_GRAPH:
                        SPARSE_GRAPH[inter].append((curr, i))
                    else:
                        SPARSE_GRAPH[inter] = [(parent, i)]

                    if parent in SPARSE_GRAPH:
                        SPARSE_GRAPH[parent].append((curr, i-1))
                    else:
                        SPARSE_GRAPH[parent] = [(curr, i-1)]
                        
            else : #is just a connector node & can be added onto the graph
                for c in DENSE_GRAPH[curr]:
                    #parent is the connect node it came from
                    if i==0: parent = curr
                    else: parent = parent

                    if not(c in visited):                     
                        st.append((c, i+1, parent)) #add the children with 1 level more {can use level 1 children as connectors}

        # connectors = DENSE_GRAPH[inter]

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

    connectSPARSE()
    print(SPARSE_GRAPH)
    plot_graph(SPARSE_GRAPH)
    plt.show()




if __name__ == '__main__':
    main()