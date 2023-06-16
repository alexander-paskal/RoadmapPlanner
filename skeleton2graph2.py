import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict




class SparseGraph():
    def __init__(self, dense_graph):
        # (u1, v1): {set of children}

        self.graph = defaultdict(list)
        self.weights = {}

        self.dense_graph = dense_graph
        self.intersections = self.extract_intersections(dense_graph)

        for node in self.intersections:
            for child in dense_graph[node]:
                # if child not in intersections:
                int1 = node
                con1 = child
                try:
                    con2, int2, weight = self._run_to_end(con1, int1)
                except:
                    print("fail")
                    continue

                self.graph[int1].append(con1)
                self.weights[(int1, con1)] = np.linalg.norm(np.array(int1) - np.array(con1))

                self.graph[con1].append(con2)
                self.weights[(con1, con2)] = weight

                self.graph[con2].append(int2)
                self.weights[(con2, int2)] = np.linalg.norm(np.array(int2) - np.array(con2))


    def extract_intersections(self, dense_graph):
        intersections = []
        for node, children in dense_graph.items():
            if len(children) > 2 or len(children) == 1:
                intersections.append(node)
        return set(intersections)

    def _run_to_end(self, node, prev, weight=0):

        if node in self.intersections:
            if prev in self.intersections:
                weight += np.linalg.norm(np.array(node) - np.array(prev))
            return prev, node, weight

        weight += np.linalg.norm(np.array(node) - np.array(prev))

        for child in self.dense_graph[node]:
            if child != prev:
                return self._run_to_end(child, node, weight=weight)

    def serialize(self):
        """condenses to single dictionary node: (child, weight) """

        new_graph = defaultdict(list)
        for node, children in self.graph.items():
            for child in children:
                weight = self.weights[(node, child)]
                new_graph[node].append((child, weight))

        return new_graph


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