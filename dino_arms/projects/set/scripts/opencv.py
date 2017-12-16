import numpy as np
from sklearn.cluster import KMeans
import cv2

"""
Authors: Cassandra Overney and Enmo Ren
Purpose: Takes an image and detects the 12 cards within it. It also returns the coordinates of the cards. 
Run: This file is imported in Game.py. In order to run it you need to download sklearn. To do so, type: pip install -U scikit-learn

"""

coords = []
matched_coords = {}

###############################################################################
# Attribute Detection
###############################################################################
def preprocess(img):
    """
    Processes an image and returns its threshold version

    :param img: Cropped out image of a single card
    :return: Threshold of the given image
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 2)
    #thresh = cv2.adaptiveThreshold(blur, 255, 1, 1, 11, 1)
    #thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
    return thresh

def preprocess2(img):
    """
    Processes an image and returns its threshold version using Otsu's method

    :param img: Cropped out image of a single card
    :return: Threshold of the given image
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    ret3, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return thresh

def find_num_shape(img):
    """
    Takes a card image and determines if the card is dashed or not. If the card is not dashed, then it
    returns the number of shapes on the card.

    :param img: Cropped out image of a single card
    :return: 'd' if the card is dashed and a number (1,2, or 3) if the card is not dashed
    """
    features = preprocess(img)
    # cv2.imshow("original img", img)
    # cv2.imshow("edited img", features)
    # cv2.waitKey(1000)
    # return sorted(training.values(), key=lambda x: imgdiff(x[1], features), reverse=True)[0][0] #for ssim
    image, contours, hierarchy = cv2.findContours(features, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[1:11]
    i = 0
    count = 0
    w_array = []
    h_array = []
    for c in contours:
        peri = cv2.arcLength(c, True)
        x, y, w, h = cv2.boundingRect(c)
        if h < 80:
            count = count + 1
            if count > 3:
                break
        # print(w, h)
        # cv2.drawContours(img, c, -1, (0, 255, 0), 3)
        # cv2.imshow("window title", img)
        # cv2.waitKey(500)

        w_array.append(w)
        h_array.append(h)
        i = i + 1

    # print("i", i)
    # print("w_array", w_array)
    # print("h_array", h_array)
    if i > 9:
        # this means that the card is dashed
        # print("dashed")
        return 'd'

    else:
        j = 0
        for i, w in enumerate(w_array):
            if w < 90 or h_array[i] < 90:
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
    """
    Takes a card image and the information from the find_num_shape function and if the card is dashed, it computes
    the number of cards. If the card is not dashed, it computes whether it is solid or empty in fill. This
    function uses the OTSU thresholding method

    :param info: Information returned from the find_num_shape function
    :param c: Cropped out image of a single card
    :return: Updated information (number and fill) for each card
    """
    #USES OTSU!!!
    features = preprocess2(c)
    # cv2.imshow("original img", c)
    # cv2.imshow("edited img", features)
    # cv2.waitKey(1000)
    image, contours, hierarchy = cv2.findContours(features, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if 'd' in info:
        #print("dashed")
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[1:8]
        w_array = []
        h_array = []
        y_array = []
        for con in contours:
            peri = cv2.arcLength(con, True)
            x, y, w, h = cv2.boundingRect(con)
            # print("x", x, "y", y)
            y_array.append(y)

            # cv2.drawContours(c, con, -1, (0, 255, 0), 3)
            # cv2.imshow("Find fill", c)
            # cv2.waitKey(500)

            w_array.append(w)
            h_array.append(h)

        # print(y_array)
        y_array_sort = sorted(y_array)
        y_diff = y_array_sort[-1] - y_array_sort[0]
        # print(y_diff)
        if y_diff < 200:
            # print("d_1")
            return '103'
        elif y_diff > 500:
            # print("d_3")
            return '303'
        else:
            # print("d_2")
            return '203'

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
    """
    Takes the information from the find_fill function and computes the type of shape of the card

    :param info: Information from the find_fill function that contains the number of shapes and the fill type
    :param c: Cropped out image of a single card
    :return: Updated information of the card with number, fill, and type of shape
    """
    features = preprocess2(c)
    # cv2.imshow("original img", c)
    # cv2.waitKey(1000)
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
        blank_image = np.zeros((1008, 633, 3), np.uint8)
        cv2.drawContours(blank_image, contours[0], -1, (0, 255, 0), 3)
        edges = cv2.Canny(blank_image, 50, 150, apertureSize=3)
        # cv2.imshow('edges', edges)
        # cv2.waitKey(1000)
        i = 30
        while i < 60:
            lines = cv2.HoughLines(edges, 1, (i * np.pi / 180), 100)
            #print(lines)
            if lines is not None:
                # for rho, theta in lines[0]:
                #         # print("theta", theta)
                #         a = np.cos(theta)
                #         b = np.sin(theta)
                #         x0 = a * rho
                #         y0 = b * rho
                #         x1 = int(x0 + 1000 * (-b))
                #         y1 = int(y0 + 1000 * (a))
                #         x2 = int(x0 - 1000 * (-b))
                #         y2 = int(y0 - 1000 * (a))
                #
                #         cv2.line(c, (x1, y1), (x2, y2), (0, 0, 255), 2)
                # cv2.imshow('line', c)
                # cv2.waitKey(500)
                for rho, theta in lines[0]:
                    if theta != 0:
                        # print("diamond", num + '3' + info[2])
                        return num + '3' + info[2]  # diamond
            i = i + 1
        # print("wavy", num + '2' + info[2])
        return num + '2' + info[2] #wavy


def find_color3(info, image):
    """
    Takes the information from the find_shape function and computes the color of the card

    :param info: Information from the find_shape function that contains the number of shapes, fill type, and type of shape
    :param image: Cropped out image of a single card
    :return: Updated information of the card with all four attributes determined
    """
    image = image[300:700, 200:300]
    # cv2.imshow("title4", image)
    # cv2.waitKey(1000)
    h = image.shape[0]
    w = image.shape[1]
    colored_pixB = []
    colored_pixG = []
    colored_pixR = []
    for y in range(0,h):
        for x in range(0,w):
            pix = image[y,x]
            if pix[0] < 200 and pix[1] < 200 and pix[2] < 200:
                colored_pixB.append(pix[0])
                colored_pixG.append(pix[1])
                colored_pixR.append(pix[2])
    average_colorB = sum(colored_pixB) / float(len(colored_pixB))
    # print("average color blue", average_colorB)
    average_colorG = sum(colored_pixG) / float(len(colored_pixG))
    # print("average color green", average_colorG)
    average_colorR = sum(colored_pixR) / float(len(colored_pixR))
    # print("average color red", average_colorR)

    if average_colorG > average_colorB and average_colorG > average_colorR:
        # print("green")
        return '1' + info #+ 'green'
    elif average_colorR > average_colorG and average_colorR > average_colorB:
        # print("red")
        return '3' + info  # + 'red'
    else:
        # print("purple")
        return '2' + info  # + 'purple'


def finalize(index, info):
    """
    Populates the matched_coords dictionary to match each card with its center coordinate

    :param index: Index number of the card
    :param info: Information from find_color2 which is used as a key in the dictionary
    :return: None
    """
    matched_coords[info] = coords[index]

def finalize2():
    """
    Sorts the matched_coords dictionary by center coordinate values and returns an array of the keys
    :return: Array of the card information sorted by position
    """
    sorted_coord = sorted(matched_coords.items(), key=lambda k: [k[1], k[0]])
    # print("sorted", sorted_coord)
    image_array = []
    for i, thing in enumerate(sorted_coord):
        image_array.append(sorted_coord[i][0])
    #print(image_array)
    return image_array


###############################################################################
# Card Extraction
###############################################################################
def getCards(im, numcards=12):
    """
    Extracts a card image from the main image

    :param im: Main image with all 12 cards combined
    :param numcards: Number of cards to extract
    :return: Extracted card image
    """
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (1, 1), 1000)
    flag, thresh = cv2.threshold(blur, 120, 255, cv2.THRESH_BINARY)

    image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:numcards]

    for card in contours:

        # Find perimeter of card and use it to approximate corner points
        # cv2.drawContours(im, card, -1, (0, 255, 0), 3)
        # cv2.imshow("window title", im)
        # cv2.waitKey(500)

        peri = cv2.arcLength(card, True)
        approx = cv2.approxPolyDP(card, 0.01 * peri, True)
        pts = np.float32(approx)

        # Find width and height of card's bounding rectangle
        x, y, w, h = cv2.boundingRect(card)

        # Find center point of card by taking x and y average of the four corners.
        average = np.sum(pts, axis=0) / len(pts)
        cent_x = int(average[0][0])
        cent_y = int(average[0][1])
        # print("cent x", cent_x/100, "cent y", cent_y/100)

        coords.append((cent_x/100, cent_y/100))

        # Warp card into 211*336 flattened image using perspective transform
        warp = flattener(im, pts, w, h)

        # cv2.imshow("Card Detector", warp)
        # cv2.waitKey(2000)

        yield warp


def flattener(image, pts, w, h):
    """
    Flattens an image of a card into a top-down 633x1008 perspective.
    Returns the flattened, re-sized image.

    :param image: Extracted card image from getCards()
    :param pts: Corner pts of a single card
    :param w: Width of a single card
    :param h: Height of a single card
    :return: Flattened image of a single card
    """
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
    """
    Takes the main image and runs the necessary methods to detect the attributes of all 12 card.

    :param im: Main image taken by Edwin
    :return: Array of card information sorted by coordinate positions
    """
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
    #   card = find_fill(cards1[i],c)
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
    #     card = find_color3(cards3[i], c)
    #     cv2.imshow(str(card), c)
    #     cv2.waitKey(2000)

    cards4 = [find_color3(cards3[i], c) for i, c in enumerate(getCards(im, num_cards))]
    #print(cards4)

    for i, c in enumerate(getCards(im, num_cards)):
        finalize(i, cards4[i])

    return finalize2()