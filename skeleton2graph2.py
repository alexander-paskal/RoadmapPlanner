import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict


DENSE_GRAPH = defaultdict(list)
    #(u1, v1): {set of children}

SPARSE_GRAPH = defaultdict(list)
SPARSE_WEIGHTS = {}


def extract_intersections(dense_graph):
    intersections = []
    for node, children in dense_graph.items():
        if len(children) > 2:
            intersections.append(node)
    return set(intersections)


def _run_to_end(node, prev, intersections=None, weight=0):

    if intersections is None:
        intersections = extract_intersections(DENSE_GRAPH)

    if node in intersections:
        if prev in intersections:
            weight += np.linalg.norm(np.array(node) - np.array(prev))
        return prev, node, weight

    weight += np.linalg.norm(np.array(node) - np.array(prev))

    for child in DENSE_GRAPH[node]:
        if child != prev:
            return _run_to_end(child, node, intersections=intersections, weight=weight)



def construct_sparse_graph():
    intersections = extract_intersections(DENSE_GRAPH)

    for node in intersections:
        for child in DENSE_GRAPH[node]:
            # if child not in intersections:
            int1 = node
            con1 = child
            try:
                con2, int2, weight = _run_to_end(con1, int1)
            except:
                print("fail")
                continue

            SPARSE_GRAPH[int1].append(con1)
            SPARSE_WEIGHTS[(int1, con1)] = np.linalg.norm(np.array(int1) - np.array(con1))

            SPARSE_GRAPH[con1].append(con2)
            SPARSE_WEIGHTS[(con1, con2)] = weight

            SPARSE_GRAPH[con2].append(int2)
            SPARSE_WEIGHTS[(con2, int2)] = np.linalg.norm(np.array(int2) - np.array(con2))

            # plot_graph(SPARSE_GRAPH)
            # print(len(SPARSE_GRAPH))
            # plt.show()
            #

def plot_graph(graph, weights=None):
    for (u, v) in graph:
        plt.scatter(u, v, c="red", s=6)

    for (u, v), children in graph.items():


        for (u1, v1) in children:
            linewidth = 1
            if weights is not None:
                linewidth = SPARSE_WEIGHTS[(u, v), (u1, v1)] / 50
                print(linewidth)
            plt.plot([u, u1], [v, v1], c="blue", alpha=0.5, linewidth=linewidth)

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
    #
    # plt.imshow(np.rot90(arr, axes=(0, 1)))
    plot_graph(DENSE_GRAPH)
    # #
    # # ints = extract_intersections(DENSE_GRAPH)
    # # plt.scatter(*np.array(list(ints)).T, c="green", s=30)
    # # print(len(ints))
    plt.show()

    construct_sparse_graph()
    plot_graph(SPARSE_GRAPH, weights=SPARSE_WEIGHTS)
    # plot_graph(SPARSE_GRAPH)
    print(len(SPARSE_GRAPH))
    plt.show()



if __name__ == '__main__':
    main()