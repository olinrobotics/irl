#!/usr/bin/env python
"""
Plot helpers to visualize the structure
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np


def plot_cube3d(coords):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(coords[:, 0], coords[:, 1], coords[:, 2], zdir='z', c='red')
    plt.axis('equal')
    plt.show()


def plot_cube2d(cubes):
    plt.plot(cubes[:, 0], cubes[:, 2], 'ro')
    plt.axis('equal')
    plt.show()


def reduce_coords(coords):
    c = []
    for i, coord in enumerate(coords):
        x, y, z = coord
        y = -y
        if not np.math.isnan(x) and z < 1:
            if i % 10 != 0:
                continue
            c.append(coord)

    return np.asarray(c)


if __name__ == '__main__':
    coords = np.loadtxt('coords_1.txt', dtype=float)
    coords = reduce_coords(coords)
    plot_cube3d(coords)
