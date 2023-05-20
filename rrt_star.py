"""
Contains implementation of RRT* algorithm
"""
from two_link import TwoLink
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialgeometry import Cuboid
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Rectangle



def _reconstruct_path(start, end, parents):
    state = end
    path = [end]
    while True:

        if state == start:
            return path[::-1]

        state = parents[state]
        path.append(state)


class Queue(list):
    def __init__(self, *args, key=None, **kwargs):
        super().__init__(*args, **kwargs)
        self._key = key
        self.sort(key=self._key)
        self._set = set(self)

    def append(self, element):
        super().append(element)
        self.sort(key=self._key)
        self._set.add(element)

    def __contains__(self, item):
        if item in self._set:
            return True
        return False

    def pop(self, index=0):
        item = super().pop(index)
        self._set.remove(item)
        return item


def djikstra(graph, start, end, cost):
    """
    Taken from my own library: https://github.com/alexander-paskal/PySOP
    Performs djikstra's algorithm to find shortest path from start to end in some graph
    :param graph: a Dict mapping states to Lists of children
    :param start: the state at which to start
    :param end: the state at which to end
    :param cost: a function(start, state) returning the cost from start to state
    :return: path, a List of states from start to finish
    :return: explored, a Set of states that have been visited by the algorithm
    """
    frontier = Queue([start], key=lambda x: cost(start, x))
    explored = set()
    solution = [start]
    parents = {}

    while True:
        if len(frontier) == 0:
            raise RuntimeError("encountered empty frontier queue")
        state = frontier.pop()
        if state == end:
            return _reconstruct_path(start, end, parents), explored
        explored.add(state)
        for child in graph[state]:
            if child not in explored and child not in frontier:
                frontier.append(child)
                parents[child] = state
            elif child in frontier:
                ix = frontier.index(child)
                f_child = frontier[ix]
                f_value = cost(start, f_child)
                n_value = cost(start, child)
                if f_value > n_value:
                    frontier.pop(ix)
                    frontier.append(child)
                    solution[-1] = child
                    parents[child] = state


def plot_tree(tree_to_plot):
    for p in tree_to_plot.keys():

        for c in tree_to_plot[p]:
            plt.plot([c[0], p[0]], [c[1], p[1]], c="red")

    ps = np.vstack([np.array(k) for k in tree_to_plot.keys()])

    plt.scatter(*ps.T, c="green")



class Tree:
    def __init__(self, root):
        self.root = root
        self.costs = {root: 0}
        self.parents = {root: None}
        self.children = {root: set()}

    def add_child(self, node, parent):
        self.children[parent].add(node)
        self.children[node] = set()
        cost_pc = self.compute_cost(node, parent)
        self.costs[node] = self.costs[parent] + cost_pc
        self.parents[node] = parent

    def remove_child(self, node):
        self.children.pop(node)
        parent = self.parents[node]
        self.children[parent].remove(node)
        self.parents.pop(node)
        self.costs.pop(node)

    def compute_cost(self, node1, node2):
        return np.linalg.norm(
            np.array(node1) - np.array(node2)
        )

    def json(self):
        tree = {
            parent: list(children) for parent, children in self.children.items()
        }
        return tree

    def knn(self, query, k=5):
        if not isinstance(query, np.ndarray):
            query = np.array(query)

        if len(self.children) == 1:
            return [self.root]

        nodes = np.array(list(self.children.keys()))
        dists = np.linalg.norm(nodes - query[None, :], axis=1)
        ind = min(k, dists.shape[0])
        ams = np.argsort(dists)
        k_nodes = nodes[ams[:ind]]
        return k_nodes

    def path_to_root(self, node):
        path = [node]
        while True:
            node = self.parents[node]
            path.append(node)

            if node == self.root:
                break

        return path

    def update_cost(self, node):
        parent = self.parents[node]
        self.costs[node] = self.costs[parent] + self.compute_cost(node, parent)

        for child in self.children[node]:
            self.update_cost(child)

def sample_map():
    qs = np.random.random(2) * np.pi * 2
    qs = qs - np.pi
    qs = qs * 0.97
    return np.round(qs, 2)


def check_valid_path(start, goal, robot):
    qs_on_path = interp_path(start, goal)
    # plt.scatter(*np.array(qs_on_path).T)
    # plt.show()
    obstacles = get_obstacles()
    collisions = [check_collision(q, robot, *obstacles) for q in qs_on_path]
    return not np.any(collisions)


def interp_path(start, goal):
    vec = goal - start
    parts = np.linspace(0, 1, 30)

    qs_interp = []
    for part in parts:
        qs_interp.append(start + part * vec)

    return qs_interp


def check_collision(q, robot, *obstacles):
    collision = False
    for obs in obstacles:
        c = robot.iscollided(q, obs)
        collision = collision or c

    return collision


_OBS = None
def get_obstacles():
    a = 30
    b = 1
    c = 35
    d = 5
    e = 11
    f = 28
    global _OBS
    if _OBS is None:

        obstacle1 = Cuboid(scale=[b, a, 1], pose=SE3(-e - b / 2, d + a / 2, 0.5), collision=True)
        obstacle2 = Cuboid(scale=[b, c, 1], pose=SE3(-e - b / 2, -c / 2, 0.5), collision=True)
        obstacle3 = Cuboid(scale=[b, c, 1], pose=SE3(e + b / 2, c / 2, 0.5), collision=True)
        obstacle4 = Cuboid(scale=[b, a, 1], pose=SE3(e + b / 2, -d - a / 2, 0.5), collision=True)
        _OBS = [obstacle1, obstacle2, obstacle3, obstacle4]

    return _OBS


def RRT_star(tree, goal, robot, k=5):



    sample = sample_map()

    # Nearest neighbors
    k_nodes = tree.knn(sample, k)
    k_node_costs = [tree.costs[tuple(n)] for n in k_nodes]

    argmins = np.argsort(k_node_costs)

    valid = False
    for argmin in argmins:
        closest_node = tuple(k_nodes[argmin])
        valid = check_valid_path(closest_node, sample, robot)

        if valid:
            node_hash = tuple(sample)
            tree.add_child(node_hash, closest_node)
            break
    # look for goal node
    success = False
    if tuple(goal) in tree.children:
        success = True
    elif valid and check_valid_path(sample, goal, robot):
        tree.add_child(tuple(goal), node_hash)
        success = True


    # Rewire tree
    if valid:
        # for argmin in argmins:
        #     node_to_check = tuple(k_nodes[argmin])
        #     if node_to_check == closest_node or node_to_check not in tree.children:
        #         continue
        #
        #     cost_to_check = tree.costs[node_to_check]
        #     cost_to_compare = tree.costs[node_hash] + tree.compute_cost(node_to_check, node_hash)
        #     if cost_to_check > cost_to_compare and check_valid_path(np.array(node_to_check), np.array(node_hash), robot):
        #         print("rewire")
        #         tree.parents[node_to_check] = node_hash
        #         tree.update_cost(node_to_check)

        neighbors = tree.knn(sample)
        sample_hash = tuple(sample)

        for neighbor in neighbors:
            if not check_valid_path(np.array(neighbor), sample, robot):
                continue
            neighbor = tuple(neighbor)
            cost_to_check = tree.costs[neighbor]
            cost_to_compare = tree.costs[sample_hash] + tree.compute_cost(neighbor, sample_hash)

            if cost_to_check > cost_to_compare:
                print("rewire")
                children = tree.children[neighbor]
                tree.remove_child(neighbor)
                tree.add_child(neighbor, sample_hash)
                for child in children:
                    tree.children[neighbor].add(child)
                tree.update_cost(neighbor)


    return sample, closest_node, valid, success





if __name__ == '__main__':
    t = Tree((-19.43, 4.62))

    t.add_child((1, 1), t.root)
    robot = TwoLink()
    while True:
        RRT_star(t, np.array((2,2)), robot)

        plot_tree(t.json())
        # plt.show()
        plt.draw()
        plt.pause(1)
        plt.cla()