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


        self.paths = {}


        for node in self.intersections:
            for child in dense_graph[node]:
                # if child not in intersections:
                int1 = node
                con1 = child
                try:
                    con2, int2, weight, path = self._run_to_end(con1, int1)
                except:
                    print("fail")
                    continue

                self.graph[int1].append(con1)
                self.weights[(int1, con1)] = np.linalg.norm(np.array(int1) - np.array(con1))
                self.paths[(int1, con1)] = [int1, con1]

                self.graph[con1].append(con2)
                self.weights[(con1, con2)] = weight
                self.paths[(con1, con2)] = path

                self.graph[con2].append(int2)
                self.weights[(con2, int2)] = np.linalg.norm(np.array(int2) - np.array(con2))
                self.paths[(con2, int2)] = [con2, int2]

    def extract_intersections(self, dense_graph):
        intersections = []
        for node, children in dense_graph.items():
            if len(children) > 2 or len(children) == 1:
                intersections.append(node)
        return set(intersections)

    def _run_to_end(self, node, prev, weight=0, path=None):
        if path is None:
            path = []

        path.append(node)

        if node in self.intersections:
            if prev in self.intersections:
                weight += np.linalg.norm(np.array(node) - np.array(prev))
            return prev, node, weight, path

        weight += np.linalg.norm(np.array(node) - np.array(prev))


        for child in self.dense_graph[node]:
            if child != prev:
                return self._run_to_end(child, node, weight=weight, path=path)

    def _walk_to_end(self, node, prev, weight=0):
        if node not in self.intersections:
            yield node

            for child in self.dense_graph[node]:
                if child != prev:
                    yield from self._walk_to_end(child, node, weight=weight)

    def serialize(self):
        """condenses to single dictionary node: (child, weight) """

        new_graph = defaultdict(list)
        for node, children in self.graph.items():
            for child in children:
                weight = self.weights[(node, child)]
                new_graph[node].append((child, weight))

        return new_graph

    def connectors(self):
        connectors = dict()
        for node, children in self.graph.items():
            for child in children:

                next_node, _, _ = self._run_to_end(child, node)

                connectors[(node, next_node)] = child

        return connectors

    def densify_path(self, path):
        dense_path = []
        # connectors = self.connectors()
        for i in range(1, len(path) - 1):
            node = path[i]
            next_node = path[i + 1]

            dense_path.extend(self.paths[(node, next_node)])

        return dense_path