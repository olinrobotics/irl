"""
Card Recognition using OpenCV
Code from the blog post
http://arnab.org/blog/so-i-suck-24-automating-card-games-using-opencv-and-python

Usage:

  ./card_img.py filename num_cards training_image_filename training_labels_filename num_training_cards

Example:
  ./card_img.py test.JPG 4 train.png train.tsv 56

Note: The recognition method is not very robust; please see SIFT / SURF for a good algorithm.

"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import argparse
import utils
from skimage.measure import compare_ssim as ssim
from sklearn.cluster import KMeans
sys.path.insert(0, "/usr/local/lib/python2.7/site-packages/")
import cv2
import operator


###############################################################################
# Utility code from
# http://git.io/vGi60A
# Thanks to author of the sudoku example for the wonderful blog posts!
###############################################################################
coords = []
matched_coords = {}

def rectify(h):
    h = h.reshape((4, 2))
    hnew = np.zeros((4, 2), dtype=np.float32)

    add = h.sum(1)
    hnew[0] = h[np.argmin(add)]
    hnew[2] = h[np.argmax(add)]

    diff = np.diff(h, axis=1)
    hnew[1] = h[np.argmin(diff)]
    hnew[3] = h[np.argmax(diff)]

    return hnew


###############################################################################
# Image Matching
###############################################################################
def preprocess(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 2)
    #thresh = cv2.adaptiveThreshold(blur, 255, 1, 1, 11, 1)
    #thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
    # blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # ret3, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return thresh

def preprocess2(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #blur = cv2.GaussianBlur(gray, (5, 5), 2)
    #thresh = cv2.adaptiveThreshold(blur, 255, 1, 1, 11, 1)
    #thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    #thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    ret3, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return thresh

def find_num_shape(img):
    features = preprocess(img)
    # cv2.imshow("original img", img)
    # cv2.imshow("edited img", features)
    # cv2.waitKey(2000)
    # return sorted(training.values(), key=lambda x: imgdiff(x[1], features), reverse=True)[0][0] #for ssim
    image, contours, hierarchy = cv2.findContours(features, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[1:10]
    i = 0
    w_array = []
    h_array = []
    for c in contours:
        peri = cv2.arcLength(c, True)
        x, y, w, h = cv2.boundingRect(c)
        if w and h < 80:
            break
        # print(w, h)
        # cv2.drawContours(img, c, -1, (0, 255, 0), 3)
        # cv2.imshow("window title", img)
        # cv2.waitKey(1500)

        w_array.append(w)
        h_array.append(h)
        i = i + 1

    # print("i", i)
    # print("w_array", w_array)
    # print("h_array", h_array)
    if i > 7:
        # this means that the card is dashed
        # print("dashed")
        return 'd'

    else:
        j = 0
        for i, w in enumerate(w_array):
            if w < 80 or h_array[i] < 80:
                break
            else:
                j = j + 1

        # print("j", j)
        i = j
        if i >= 5:
            # print('3')
            return '3'

        elif i < 5 and i >= 3:
            # print('2')
            return '2'

        else:
            # print('1')
            return '1'

def find_fill(info, c):
    #USEs OTSU!!!
    features = preprocess2(c)
    # cv2.imshow("original img", c)
    # # cv2.imshow("edited img", features)
    # cv2.waitKey(2000)
    image, contours, hierarchy = cv2.findContours(features, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if 'd' in info:
        #print("dashed")
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[1:8]
        i = 0
        w_array = []
        h_array = []
        for con in contours:
            peri = cv2.arcLength(con, True)
            x, y, w, h = cv2.boundingRect(con)
            if w < 170 or h < 170:
                break
            # print(w, h)
            # cv2.drawContours(c, con, -1, (0, 255, 0), 3)
            # cv2.imshow("Find fill", c)
            # cv2.waitKey(1000)

            w_array.append(w)
            h_array.append(h)
            i = i + 1
        # print(len(contours))
        #print(i)
        if i >= 5:
            # print('d_3')
            return '303'

        elif i < 5 and i >= 3:
            # print('d_2')
            return '203'

        else:
            # print('d_1')
            return '103'
    else:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[1:7]
        i = 0
        w_array = []
        h_array = []
        for con in contours:
            peri = cv2.arcLength(con, True)
            x, y, w, h = cv2.boundingRect(con)
            if w and h < 150:
                break
            # print(w, h)
            # cv2.drawContours(c, con, -1, (0, 255, 0), 3)
            # cv2.imshow("Find fill", c)
            # cv2.waitKey(1000)

            w_array.append(w)
            h_array.append(h)
            i = i + 1
        #print(i)
        if info == str(i):
            # print("solid")
            return info+'01'
        else:
           # print("empty")
           return info + '02'


def find_shape(info, c):
    features = preprocess2(c)
    # cv2.imshow("original img", c)
    # cv2.waitKey(2000)
    image, contours, hierarchy = cv2.findContours(features, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    num = info[0]
    totallen = 0
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[1:(int(num)+1)]
    for con in contours:
        # cv2.drawContours(c, con, -1, (0, 255, 0), 3)
        # cv2.imshow("window title", c)
        # cv2.waitKey(5000)
        peri = cv2.arcLength(con, True)
        approx = cv2.approxPolyDP(con, 0.04 * peri, True)
        totallen = totallen + len(approx)
    length = float(totallen / int(num))
    #print(length)
    if length > 5:
        # print("circle", num + '1' + info[2])
        return num + '1' + info[2] #circle
    elif length == 5:
        # print("wavy", num + '2' + info[2])
        return num + '2' + info[2] #wavy
    else:
        gray = cv2.cvtColor(c, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        # cv2.imshow('edges', edges)
        # cv2.waitKey(2000)
        minLineLength = 100
        maxLineGap = 0
        i = 10
        while i < 60:
            lines = cv2.HoughLinesP(edges, 1, (i * np.pi / 180), 100, minLineLength, maxLineGap)
            #lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)
            #print(lines)
            if lines is not None:
            #     #print(lines)
            #     for x in range(0, len(lines)):
            #         for x1, y1, x2, y2 in lines[x]:
            #             cv2.line(c, (x1, y1), (x2, y2), (0, 255, 0), 2)
            #
            #     cv2.imshow('line', c)
            #     cv2.waitKey(3000)
            #     print("diamond", num + '3' + info[2])
                return num + '3' + info[2] #diamond
            i = i + 1
        # print("wavy", num + '2' + info[2])
        return num + '2' + info[2] #wavy


def find_color2(info, image):
    image = cv2.resize(image, (633, 1008))
    im = image[300:700, 200:300]
    # cv2.imshow("title4", im)
    # cv2.waitKey(2000)
    im = im.reshape((im.shape[0] * im.shape[1], 3))
    clt = KMeans(2)
    clt.fit(im)
    #print(clt.labels_)
    bar = utils.plot_colors(clt.cluster_centers_)
    #print("bar", bar)
    a = 0
    j = 0
    differences = []
    for i, b in enumerate(bar):
        if a < b:
            a = b
            j = i

    if j == 1:
        diff1 = bar[j] - bar[0]
        diff2 = bar[j] - bar[2]
        if diff1 and diff2 > 5:
            # print("green")

            return '1' + info #+ 'green'
        else:
            # print("purple")
            return '2' + info #  + 'purple'

    elif j == 2:
        diff1 = bar[j] - bar[0]
        diff2 = bar[j] - bar[1]
        if diff1 and diff2 > 8:
            # print("red")
            return '3' + info #+ 'red'
        else:
            # print("purple")
            return '2' + info  # + 'purple'

    else:
        # print("purple")
        return '2' + info #+ 'purple'

def finalize(index, info):
    matched_coords[info] = coords[index]

def finalize2():
    sorted_coord = sorted(matched_coords.items(), key=lambda k: [k[1], k[0]])
    #print("sorted", sorted_coord)
    image_array = []
    for i, thing in enumerate(sorted_coord):
        image_array.append(sorted_coord[i][0])
    print(image_array)
    return image_array


###############################################################################
# Card Extraction
###############################################################################
def getCards(im, numcards=4):
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (1, 1), 1000)
    flag, thresh = cv2.threshold(blur, 120, 255, cv2.THRESH_BINARY)

    image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:numcards]

    for card in contours:

        # Find perimeter of card and use it to approximate corner points
        peri = cv2.arcLength(card, True)
        approx = cv2.approxPolyDP(card, 0.01 * peri, True)
        pts = np.float32(approx)

        # Find width and height of card's bounding rectangle
        x, y, w, h = cv2.boundingRect(card)

        # Find center point of card by taking x and y average of the four corners.
        average = np.sum(pts, axis=0) / len(pts)
        cent_x = int(average[0][0])
        cent_y = int(average[0][1])
        #print("cent x", cent_x/100, "cent y", cent_y/100)

        coords.append((cent_x/100, cent_y/100))

        # Warp card into 211*336 flattened image using perspective transform
        warp = flattener(im, pts, w, h)

        # cv2.imshow("Card Detector", warp)
        # cv2.waitKey(2000)

        yield warp


def flattener(image, pts, w, h):
    """Flattens an image of a card into a top-down 200x300 perspective.
    Returns the flattened, re-sized, grayed image.
    See www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/"""
    temp_rect = np.zeros((4, 2), dtype="float32")

    s = np.sum(pts, axis=2)

    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]

    diff = np.diff(pts, axis=-1)
    tr = pts[np.argmin(diff)]
    bl = pts[np.argmax(diff)]

    # Need to create an array listing points in order of
    # [top left, top right, bottom right, bottom left]
    # before doing the perspective transform

    if w <= 0.8 * h:  # If card is vertically oriented
        temp_rect[0] = tl
        temp_rect[1] = tr
        temp_rect[2] = br
        temp_rect[3] = bl

    if w >= 1.2 * h:  # If card is horizontally oriented
        temp_rect[0] = bl
        temp_rect[1] = tl
        temp_rect[2] = tr
        temp_rect[3] = br

    # If the card is 'diamond' oriented, a different algorithm
    # has to be used to identify which point is top left, top right
    # bottom left, and bottom right.

    if w > 0.8 * h and w < 1.2 * h:  # If card is diamond oriented
        # If furthest left point is higher than furthest right point,
        # card is tilted to the left.
        if pts[1][0][1] <= pts[3][0][1]:
            # If card is titled to the left, approxPolyDP returns points
            # in this order: top right, top left, bottom left, bottom right
            temp_rect[0] = pts[1][0]  # Top left
            temp_rect[1] = pts[0][0]  # Top right
            temp_rect[2] = pts[3][0]  # Bottom right
            temp_rect[3] = pts[2][0]  # Bottom left

        # If furthest left point is lower than furthest right point,
        # card is tilted to the right
        if pts[1][0][1] > pts[3][0][1]:
            # If card is titled to the right, approxPolyDP returns points
            # in this order: top left, bottom left, bottom right, top right
            temp_rect[0] = pts[0][0]  # Top left
            temp_rect[1] = pts[3][0]  # Top right
            temp_rect[2] = pts[2][0]  # Bottom right
            temp_rect[3] = pts[1][0]  # Bottom left

    maxWidth = 633
    maxHeight = 1008

    # Create destination array, calculate perspective transform matrix,
    # and warp card image
    dst = np.array([[0, 0], [maxWidth - 1, 0], [maxWidth - 1, maxHeight - 1], [0, maxHeight - 1]], np.float32)
    M = cv2.getPerspectiveTransform(temp_rect, dst)
    warp = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    warp = warp[30:990, 30:620]
    warp = cv2.resize(warp, (633, 1008))
    # cv2.imshow("cropped", warp)
    # cv2.waitKey(2000)

    return warp


def find_matches(im):
    num_cards = 12

    width = im.shape[0]
    height = im.shape[1]
    if width < height:
        im = cv2.transpose(im)
        im = cv2.flip(im, 1)

    # Debug: uncomment to see registered images
    # for i, c in enumerate(getCards(im, num_cards)):
    #     card = find_num_shape(c)
    #     cv2.imshow(str(card), c)
    #     cv2.waitKey(2000)

    cards1 = [find_num_shape(c) for c in getCards(im, num_cards)]
    #print cards1

    # Debug: uncomment to see registered images
    # for i,c in enumerate(getCards(im,num_cards)):
    #   card = find_fill(cards1[i], trainings_shape, c)
    #   cv2.imshow(str(card), c)
    #   cv2.waitKey(5000)

    cards2 = [find_fill(cards1[i], c) for i, c in enumerate(getCards(im, num_cards))]
    #print(cards2)

    # for i, c in enumerate(getCards(im, num_cards)):
    #     card = find_shape(cards2[i], trainings_shape, c)
    #     cv2.imshow(str(card), c)
    #     cv2.waitKey(3000)

    cards3 = [find_shape(cards2[i], c) for i, c in enumerate(getCards(im, num_cards))]
    #print(cards3)

    # for i, c in enumerate(getCards(im, num_cards)):
    #     card = find_color2(cards3[i], c)
    #     cv2.imshow(str(card), c)
    #     cv2.waitKey(2000)

    cards4 = [find_color2(cards3[i], c) for i, c in enumerate(getCards(im, num_cards))]
    #print(cards4)

    for i, c in enumerate(getCards(im, num_cards)):
        finalize(i, cards4[i])

    return finalize2()

