from two_link import TwoLink
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialgeometry import Cuboid
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Rectangle
from rrt_star import RRT_star, Tree
from PIL import Image
from motion_planner import MotionPlanner

class Environment:
    def __init__(self):
        self.obstacles = None
        self.robot = TwoLink()

        self._build_obstacles()

        img = Image.open('cspace_image.png').convert('L')
        img = np.array(img)
        mask = img > 128
        img[mask] = 1
        img[~mask] = 0

        config_space = np.load("cspace_hw3.npy")
        minq = np.min(config_space[1:, :], axis=1)
        maxq = np.max(config_space[1:, :], axis=1)
        self.cspace = config_space
        self.cimage = img
        self.cspace_min = minq
        self.cspace_max = maxq

    def plot_configspace(self):
        c, q1, q2 = self.cspace
        c = ~c.astype(int)
        plt.scatter(q1.flatten(), q2.flatten(), c=c.flatten(), cmap="gray")
        plt.xlabel("q1")
        plt.ylabel("q2")

    def plot_workspace(self):
        ax = plt.gca()
        ax.plot([-40, 40], [0, 0], c="red")
        ax.plot([0, 0], [-40, 40], c="red")
        for o in self.obstacles:
            center = o.T[:3, 3]
            scale = o.scale

            left_bottom_corner = center - scale / 2
            patch = Rectangle(left_bottom_corner[:2], height=scale[1], width=scale[0])
            # patch = Rectangle(left_bottom_corner[:2], height=5, width=5)
            ax.add_patch(patch)

    def plot_robot(self, q):
        result = self.robot.fkine_all(q)
        for i, pose in enumerate(result.A[:-1]):
            nextpose = result.A[i + 1]
            sx, sy = pose[:2, 3]
            ex, ey = nextpose[:2, 3]
            plt.plot([sx, ex], [sy, ey], c="green", linewidth=3, alpha=0.5)

        for i, pose in enumerate(result.A[:-1]):
            nextpose = result.A[i + 1]
            ex, ey = nextpose[:2, 3]
            plt.scatter(ex, ey, c="orange", s=30)

    def _build_obstacles(self):
        a = 30
        b = 1
        c = 35
        d = 5
        e = 11
        f = 28

        if self.obstacles is None:
            obstacle1 = Cuboid(scale=[b, a, 1], pose=SE3(-e - b / 2, d + a / 2, 0.5), collision=True)
            obstacle2 = Cuboid(scale=[b, c, 1], pose=SE3(-e - b / 2, -c / 2, 0.5), collision=True)
            obstacle3 = Cuboid(scale=[b, c, 1], pose=SE3(e + b / 2, c / 2, 0.5), collision=True)
            obstacle4 = Cuboid(scale=[b, a, 1], pose=SE3(e + b / 2, -d - a / 2, 0.5), collision=True)
            self.obstacles = [obstacle1, obstacle2, obstacle3, obstacle4]


if __name__ == '__main__':
    e = Environment()
    mp = MotionPlanner.create_from_config_image(e.cimage, q_start=e.cspace_min, q_end=e.cspace_max)
    print(mp.q_start)
    print(mp.q_end)

    start = (40, 100)
    startq = mp.pix2q(start).squeeze()
    end = (175, 200)
    endq = mp.pix2q(end).squeeze()

    # startq = (0, 0)
    # start = mp.q2pix(startq).squeeze()
    # endq = (1, 2)
    # end = mp.q2pix(endq).squeeze()

    # e.plot_configspace()
    # plt.scatter(*endq, c="red")
    # # plt.scatter(*end, c="green")
    # plt.show()
    #
    # im = e.cimage.copy()
    # print(im.shape)
    # u, v = end
    # im[u-3:u+3, v-3:v+3] = 2
    # plt.imshow(im)
    # # plt.scatter(())
    # plt.show()


    densepath = mp.generate_motion_plan(start, end, qs_out=True)

    densepath = mp.pix2q(np.load("prm_path_normal.npy"))

    # densepath = np.array(densepath)
    print()
    e.plot_configspace()
    plt.scatter(*densepath.T, c="blue")
    plt.show()
    fig, ax1 = plt.subplots()
    every = 3
    #

    while True:
        for i, q in enumerate(densepath.tolist()[::every]):
            # for ax in (ax1, ax2):
            #     ax.set_xticks([])
            #     ax.set_yticks([])
            print("rendering", i)
            plt.sca(ax1)
            plt.cla()
            plt.xticks([])
            plt.yticks([])
            e.plot_workspace()
            e.plot_robot(q)
            plt.title(f"{i}")

            plt.draw()
            plt.pause(0.01)





    # plot the robots
    # fig, (ax1, ax2) = plt.subplots(1, 2)
    # for ax in (ax1, ax2):
    #     ax.set_xticks([])
    #     ax.set_yticks([])
    #
    #
    # plt.sca(ax1)
    # e.plot_workspace()
    # e.plot_robot(startq)
    #
    # plt.sca(ax2)
    # e.plot_workspace()
    # e.plot_robot(endq)
    #
    # plt.show()




