
import sys
import numpy as np
import argparse
from classes import PRMController, Obstacle, Utils
import time

def main(args):

    parser = argparse.ArgumentParser(description='PRM Path Planning Algorithm')
    parser.add_argument('--numSamples', type=int, default=1000, metavar='N',
                        help='Number of sampled points')
    args = parser.parse_args()

    numSamples = args.numSamples

    env = np.load("cspace.npy")

    current = [100,100]
    destination = [161, 24]

    print("Current: {} Destination: {}".format(current, destination))


    utils = Utils()
    utils.drawMap(env, current, destination)
    #print(env.shape)
    #assert 1==0

    prm = PRMController(numSamples, env, current, destination)
    # Initial random seed to try
    initialRandomSeed = 0
    prm.runPRM(initialRandomSeed)


if __name__ == '__main__':
    main(sys.argv)
