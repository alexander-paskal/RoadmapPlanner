import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict


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



DENSE_GRAPH = defaultdict(list)
    #(u1, v1): {set of children}

SPARSE_GRAPH = {}



def extract_intersections(dense_graph):
    intersections = []
    for node, children in dense_graph.items():
        if len(children) > 2:
            intersections.append(node)
    return intersections


def plot_dense_graph():
    for (u, v) in DENSE_GRAPH:
        plt.scatter(u, v, c="red", s=6)

    for (u, v), children in DENSE_GRAPH.items():
        for (u1, v1) in children:
            plt.plot([u, u1], [v, v1], c="blue", alpha=0.5)

    # plt.show()



def main():

    arr = np.load("skeleton.npy")


    H, W = arr.shape

    xx, yy = np.meshgrid(
            np.arange(H),
            np.arange(W)
    )

    inds = np.vstack([xx.flatten(), yy.flatten()]).T


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

                if arr[u_child, v_child] == 1:
                    DENSE_GRAPH[(u, v)].append((u_child, v_child))

    plot_dense_graph()

    ints = extract_intersections(DENSE_GRAPH)
    plt.scatter(*np.array(ints).T, c="green", s=30)
    print(len(ints))
    plt.show()



if __name__ == '__main__':
    main()