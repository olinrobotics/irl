# import the necessary packages
import numpy as np
import cv2

def plot_colors(centroids):
    # initialize the bar chart representing the relative frequency
    # of each of the colors

    # loop over the percentage of each cluster and the color of
    # each cluster
    i = 0
    for color in centroids:
        # plot the relative percentage of each cluster
        #print("color", color)
        for i,b in enumerate(color):
            if b < 200:
                return color
