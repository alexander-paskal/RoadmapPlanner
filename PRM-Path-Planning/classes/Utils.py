import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle


class Utils:
    def isWall(self, obs):
        x = [item[0] for item in obs.allCords]
        y = [item[1] for item in obs.allCords]
        if(len(np.unique(x)) < 2 or len(np.unique(y)) < 2):
            return True  # Wall
        else:
            return False  # Rectangle

    def drawMap(self, env, curr, dest):
        fig = plt.figure()

        plt.scatter(curr[1], curr[0], s=200, c='green')
        plt.scatter(dest[1], dest[0], s=200, c='green')

        plt.imshow(env)
        fig.canvas.draw()
