import cv2
import numpy as np


def find_paper(image, show_video=False):
    blur = cv2.GaussianBlur(image, (5, 5), 0)
    rgb = cv2.cvtColor(blur, cv2.COLOR_BGR2RGB)
    lower_white = np.array([150, 150, 150])
    upper_white = np.array([255, 255, 255])
    mask = cv2.inRange(rgb, lower_white, upper_white)
    paper = []
    for row, item in enumerate(mask):
        for col, value in enumerate(item):
            if value == 255:
                paper.append([row, col])
    if show_video:
        cv2.imshow('image', image)
        cv2.imshow('mask', mask)
        cv2.waitKey(0)

    return paper

