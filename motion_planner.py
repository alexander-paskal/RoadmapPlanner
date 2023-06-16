from skeletonization import skeletonize2d
import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt
from sparse_graph import SparseGraph
from a_star import a_star_SPARSE as a_star
from copy import deepcopy


class MotionPlanner:

    def __init__(self):
        self.config_image = None
        self.sdf_im = None
        self.skeleton_im = None

        self.sparse_graph = None
        self.dense_graph = None

        self.q_start = None
        self.q_end = None


    @classmethod
    def create_from_config_image(cls, config_image, q_start=(0, 0), q_end=(1, 1)):
        instance = cls.__new__(MotionPlanner)
        instance.__init__()
        instance.config_image = config_image
        instance.q_start = np.array(q_start)
        instance.q_end = np.array(q_end)

        instance.skeleton_im, instance.sdf_im = cls.skeletonize_config_image(config_image)
        instance.dense_graph = cls.dense_graph_from_skeleton(instance.skeleton_im)
        instance.sparse_graph = cls.sparse_graph_from_dense_graph(instance.dense_graph)
        return instance

    @staticmethod
    def skeletonize_config_image(config_image):
        skeleton, sdf = skeletonize2d(config_image)
        return skeleton, sdf

    @staticmethod
    def dense_graph_from_skeleton(skeleton_im):
        """
        Converts skeleton image to a dense graph
        :param skeleton_im:
        :return:
        """
        arr = skeleton_im
        H, W = arr.shape

        xx, yy = np.meshgrid(
            np.arange(H),
            np.arange(W)
        )

        # for each pixel nx2
        inds = np.vstack([xx.flatten(), yy.flatten()]).T

        # how many of all pixels are points(skeleton) in the graph
        mask = arr.T.flatten().astype(bool)

        dense_nodes = inds[mask]

        dense_graph = defaultdict(list)

        for node in dense_nodes:
            u, v = node

            for i in (-1, 0, 1):
                for j in (-1, 0, 1):
                    if i == 0 and j == 0:
                        continue

                    u_child = u + i
                    v_child = v + j

                    # iterate thru every neighbor
                    if arr[u_child, v_child] == 1:
                        dense_graph[(u, v)].append((u_child, v_child))

        return dense_graph

    @staticmethod
    def sparse_graph_from_dense_graph(dense_graph):
        return SparseGraph(dense_graph)

    def pix2q(self, pixels):

        pixels = np.array(pixels)

        if len(pixels.shape) == 1:
            pixels = pixels.reshape((-1, 2))

        u, v = pixels.T
        H, W = self.config_image.shape

        urat = (u+1)/H
        vrat = (v+1)/W

        xrange, yrange = self.q_end - self.q_start

        q1 = vrat * xrange + self.q_start[0]
        q2 = (1 - urat) * yrange + self.q_start[1]

        return np.vstack([q1.squeeze(), q2.squeeze()]).T.squeeze()

    def q2pix(self, qs):

        qs = np.array(qs)

        if len(qs.shape) == 1:
            qs = qs.reshape((-1, 2))

        q1, q2 = qs.T

        H, W = self.config_image.shape
        xstart, ystart = self.q_start
        xrange, yrange = self.q_end - self.q_start

        xrat = (q1 - xstart) /xrange
        yrat = (q2 - ystart) /yrange


        vpixel = (xrat * W).astype(int)
        upixel = ((1-yrat)*H).astype(int)

        return np.vstack([upixel.squeeze(), vpixel.squeeze()]).T.squeeze()

    def add_point_to_dense_graph(self, point):
        cur_dis = self.sdf_im[point[0], point[1]]
        cur_pos = point
        direction = np.array([[1, 1], [1, 0], [1, -1], [0, 1], [0, -1], [-1, 1], [-1, 0], [-1, -1]])
        path = [cur_pos]
        cur_pos = np.array(cur_pos)
        while tuple(cur_pos) not in self.dense_graph:
            gradient = -1
            t_pos = None
            for d in direction:
                c_pos = cur_pos + d
                c_g = self.sdf_im[c_pos[0], c_pos[1]] - self.sdf_im[cur_pos[0], cur_pos[1]]
                if c_g > gradient:
                    gradient = c_g
                    t_pos = c_pos
            cur_pos = t_pos
            path.append(tuple(t_pos))

        path.append(tuple(cur_pos))
        # reconstruct the path back to the start

        for i, node in enumerate(path[:-1]):
            next_node = path[i+1]

            if node != next_node:

                self.dense_graph[node].append(next_node)
                self.dense_graph[next_node].append(node)

    @staticmethod
    def plot_graph(graph, weights=None):
        for (u, v) in graph:
            plt.scatter(u, v, c="red", s=6)

        for (u, v), children in graph.items():

            for (u1, v1) in children:
                linewidth = 1
                if weights is not None:
                    linewidth = weights[(u, v), (u1, v1)] / 50
                    print(linewidth)
                plt.plot([u, u1], [v, v1], c="blue", alpha=0.5, linewidth=linewidth)

    def generate_motion_plan(self, start, goal, qs_in=False, qs_out=False):

        if qs_in:
            start = self.q2pix(start)
            goal = self.q2pix(goal)

        history = {
            "dense": deepcopy(self.dense_graph)
        }


        self.add_point_to_dense_graph(start)
        self.add_point_to_dense_graph(goal)

        new_sparse = self.sparse_graph_from_dense_graph(self.dense_graph)

        sparse_path, dense_path = a_star(start, goal, new_sparse.serialize(), self.dense_graph, new_sparse.connectors)
        dense_path = new_sparse.densify_path(sparse_path)
        self.dense_graph = history["dense"]

        if qs_out:
            return self.pix2q(dense_path)

        return dense_path


if __name__ == '__main__':
    from PIL import Image

    img = Image.open('cspace_image.png').convert('L')
    img = np.array(img)
    mask = img > 128
    img[mask] = 1
    img[~mask] = 0

    config_space = np.load("cspace_hw3.npy")
    minq = np.min(config_space[1:, :], axis=1)
    maxq = np.max(config_space[1:, :], axis=1)
    i = MotionPlanner.create_from_config_image(img, q_start=minq, q_end=maxq)

    start = (100, 100)
    goal = (80, 200)

    sparse_path, dense_path = i.generate_motion_plan(start, goal)

    plt.scatter(*np.array(dense_path).T)
    i.plot_graph(i.dense_graph)
    plt.scatter(*start, c="red")
    plt.scatter(*goal, c="green")
    plt.show()

