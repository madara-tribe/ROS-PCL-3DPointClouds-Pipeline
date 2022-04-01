# https://keras.io/examples/vision/pointnet/
import os, sys
import trimesh
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from matplotlib import pyplot as plt


tf.random.set_seed(1234)

def point_plot(points):
    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2])
    ax.set_axis_off()
    plt.show()

def main(path, plot=False):
    mesh = trimesh.load(path)
    mesh.show()
    points = mesh.sample(2048)
    if plot:
        point_plot(points)

if __name__=='__main__':
    path = sys.argv[1]
    main(path)
