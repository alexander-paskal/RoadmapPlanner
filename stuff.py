import numpy as np
import matplotlib.pyplot as plt


def main():
    arr = np.load("cspace_hw3.npy")
    c, q1, q2 = arr
    c = ~c.astype(int)
    plt.scatter(q1.flatten(), q2.flatten(), c=c.flatten(), cmap="gray")
    plt.xlabel("q1")
    plt.ylabel("q2")
    plt.show()

if __name__ == "__main__":
    main()
