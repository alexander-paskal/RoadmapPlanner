from skeletonization import skeletonize2d
import numpy as np
from collections import defaultdict


class MotionPlanner:

    def __init__(self):
        self.config_image = None
        self.sdf = None
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


    def pix2q(self, pixels):

        if len(pixels.shape) == 1:
            pixels = pixels.reshape((-1, 2))

        u, v = pixels.T
        H, W = self.config_image.shape

        urat = (u+1)/H
        vrat = (v+1)/W

        xrange, yrange = self.q_end - self.q_start

        q1 = vrat * xrange + self.q_start[0]
        q2 = (1 - urat) * yrange + self.q_start[1]

        return np.vstack([q1.squeeze(), q2.squeeze()]).T

    def q2pix(self, qs):

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

        return np.vstack([upixel.squeeze(), vpixel.squeeze()]).T




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


