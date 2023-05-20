from two_link import TwoLink
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialgeometry import Cuboid
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Rectangle
from rrt_star import RRT_star, Tree


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


def get_waypoint_qs(robot, *obstacles, filter=False):
    waypoints = ((0, 20), (0, -20), (-19.43, 4.62), (2, 2), (-8, 0), (-5, -5), (-9, 0))
    waypoint_qs = np.vstack([
        robot.ikine_LM(SE3.Trans(*wp, 0), ilimit=5000).q for wp in waypoints
    ])

    if filter:
        wps = []
        for wpq in waypoint_qs:
            c = False if check_collision(wpq, robot, *obstacles) else True
            if c:
                wps.append(wpq)
        waypoint_qs = np.vstack(wps)
    return waypoint_qs


def get_config_space(robot, *obstacles):
    q1, q2 = np.meshgrid(
        np.linspace(-0.97 * np.pi, 0.97 * np.pi, 97*2 + 1),
        np.linspace(-0.97 * np.pi, 0.97 * np.pi, 97 * 2 + 1),
    )

    qs = np.vstack([q1.flatten(), q2.flatten()]).T

    results = []

    for q in qs:
        collision = False
        for obs in obstacles:
            collision = collision or robot.iscollided(q, obs)

        results.append(collision)

    return np.vstack([
        np.array(results).astype(int),
        q1.flatten(),
        q2.flatten()
    ])


def interp_path(start, goal):
    vec = goal - start
    parts = np.linspace(0, 1, 5)

    qs_interp = []
    for part in parts:
        qs_interp.append(start + part * vec)

    return qs_interp


def sample_map():
    qs = np.random.random(2) * np.pi * 2
    qs = qs - np.pi
    qs = qs * 0.97
    return np.round(qs, 2)


def check_collision(q, robot, *obstacles):
    collision = False
    for obs in obstacles:
        c = robot.iscollided(q, obs)
        collision = collision or c

    return collision


def check_valid_path(start, goal, robot):
    qs_on_path = interp_path(start, goal)
    # plt.scatter(*np.array(qs_on_path).T)
    # plt.show()
    obstacles = get_obstacles()
    collisions = [check_collision(q, robot, *obstacles) for q in qs_on_path]
    return not np.any(collisions)


def closest_point(tree, point):
    nodes = np.array([q for q in tree.keys()])
    dist = np.linalg.norm(nodes - point.reshape((1, 2)), axis=1)
    min_node = nodes[np.argmin(dist)]
    return tuple(min_node)





def RRT(tree, goal, robot):
    sample = sample_map()

    closest_node = closest_point(tree, sample)

    valid = check_valid_path(closest_node, sample, robot)
    print(sample, valid)
    if valid:
        node_hash = tuple(sample)
        tree[closest_node].append(node_hash)
        tree[node_hash] = [closest_node]

    success = False

    if valid:
        if check_valid_path(sample, goal, robot):
            goal_hash = tuple(goal)
            tree[goal_hash] = [node_hash]
            tree[node_hash].append(goal_hash)
            success = True

    return sample, closest_node, valid, success


def plot_tree(tree_to_plot, **kwargs):
    for p in tree_to_plot.keys():

        for c in tree_to_plot[p]:
            plt.plot([c[0], p[0]], [c[1], p[1]], c="orange", **kwargs)

    ps = np.vstack([np.array(k) for k in tree_to_plot.keys()])

    plt.scatter(*ps.T, c="green")


def plot_path(path_to_plot, **kwargs):
    for i in range(len(path_to_plot) - 1):
        p = path_to_plot[i]
        c = path_to_plot[i + 1]
        plt.plot([c[0], p[0]], [c[1], p[1]], **kwargs)


def plot_configspace(cspace):
    c, q1, q2 = cspace
    c = ~c.astype(int)
    plt.scatter(q1.flatten(), q2.flatten(), c=c.flatten(), cmap="gray")
    plt.xlabel("q1")
    plt.ylabel("q2")


def plot_workspace(*obstacles):
    ax = plt.gca()
    ax.plot([-40, 40], [0, 0], c="red")
    ax.plot([0, 0], [-40, 40], c="red")
    for o in obstacles:
        center = o.T[:3, 3]
        scale = o.scale

        left_bottom_corner = center - scale / 2
        patch = Rectangle(left_bottom_corner[:2], height=scale[1], width=scale[0])
        # patch = Rectangle(left_bottom_corner[:2], height=5, width=5)
        ax.add_patch(patch)


def plot_fkine(q, robot):
    result = robot.fkine_all(q)
    for i, pose in enumerate(result.A[:-1]):
        nextpose = result.A[i + 1]
        sx, sy = pose[:2, 3]
        ex, ey = nextpose[:2, 3]
        plt.plot([sx, ex], [sy, ey], c="green")
        plt.scatter(ex, ey, c="orange")


def add_rectangle( xy , height , width , ax ) :
    """
    Add a rectangle patch to the environment .
    : param xy : xy corner of the left - bottom corner .
    : param height : height of the window .
    : param width : width of the window .
    : param ax : the axis on which to plot .
    """
    obj_patch = Rectangle(xy, height,width )
    ax.add_patch(obj_patch )


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


def compute_cost(node1, node2):
    return np.linalg.norm(
        np.array(node1) - np.array(node2)
    )


def run_rrt(waypoints, robot, cspace=None, its=None, break_on_success=False):
    og_start = waypoints[0]

    paths = []
    trees = []



    for i in range(waypoints.shape[0] - 1):

        start = waypoints[i]
        goal = waypoints[i + 1]

        # tree = {
        #     tuple(start): []
        # }

        tree = Tree(tuple(start))

        # while True:
        prev = False
        for i in range(its):
            s, cn, val, suc = RRT_star(tree, goal, robot, k=10)
            print(s, cn, val, suc)

            if suc and not prev:

                print("success")
                # path, _ = djikstra(tree, tuple(start), tuple(goal), compute_cost)
                firstpath = tree.path_to_root(tuple(goal))[::-1]

                # if cspace is not None:
                #     plot_configspace(cspace)
                # plot_tree(tree.json(), alpha=0.3)
                # plot_path(firstpath, c="blue")
                # # plot_path(final_path, c="red")
                # plt.xlim(-np.pi, np.pi)
                # plt.ylim(-np.pi, np.pi)
                # plt.show()
                prev = True
        final_path = tree.path_to_root(tuple(goal))[::-1]

        # if cspace is not None:
        #     plot_configspace(cspace)
        # plot_tree(tree.json(), alpha=0.3)
        # plot_path(final_path, c="red")
        # # plot_path(final_path, c="red")
        # plt.xlim(-np.pi, np.pi)
        # plt.ylim(-np.pi, np.pi)
        # plt.show()

        paths.append(final_path)
        trees.append(tree)

    return paths, trees


def RRT(tree, goal, robot):
    sample = sample_map()

    closest_node = closest_point(tree, sample)

    valid = check_valid_path(closest_node, sample, robot)

    if valid:
        node_hash = tuple(sample)
        tree[closest_node].append(node_hash)
        tree[node_hash] = [closest_node]

    success = False

    if valid:
        if check_valid_path(sample, goal, robot):
            goal_hash = tuple(goal)
            tree[goal_hash] = [node_hash]
            tree[node_hash].append(goal_hash)
            success = True

    return sample, closest_node, valid, success


def gen_trajectory(paths):
    p = []
    for path in paths:
        p.extend(path)

    final_qs = []
    for i in range(len(p) - 1):
        q1 = np.array(p[i])
        q2 = np.array(p[i + 1])

        interped = interp_path(q1, q2)
        final_qs.extend(interped)

    final_qs = np.vstack(final_qs)
    return final_qs


def main():

    robot = TwoLink()
    obstacles = get_obstacles()


    cspace = get_config_space(robot, *obstacles)
    np.save("cspace_hw3.npy", cspace)
    # plot_configspace(cspace)
    # plt.title("Config Space")
    # plt.show()

    waypoints = get_waypoint_qs(robot, *obstacles, filter=True)
    print(waypoints)
    paths, trees = run_rrt(waypoints, robot, cspace=cspace, its=500)
    qs = gen_trajectory(paths)

    waypoints = np.array([(0, 20), (0, -20), (-19.43, 4.62), (2, 2), (-5, -5)])

    # input("start gif when readY")

    fig, (ax1, ax2) = plt.subplots(1, 2)

    import time
    print("sleeping")
    time.sleep(10)

    for q in qs:
        collided = check_collision(q, robot, *obstacles)
        plt.sca(ax1)
        plot_workspace(*obstacles)
        plot_fkine(q, robot)
        plt.scatter(*waypoints.T, c="red")
        plt.title(f"Collision: {collided}")

        plt.sca(ax2)
        plot_configspace(cspace)
        plt.scatter(*q, c="orange")

        plt.draw()
        plt.pause(0.05)
        ax1.clear()
        ax2.clear()


if __name__ == '__main__':
    main()
