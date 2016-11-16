import cv2
import numpy as np
from Character import Character

def get_text_roi(frame):
    chars = []
    bound = 5
    kernel = np.ones((2,2),np.uint8)

    frame_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    frame_gray = cv2.GaussianBlur(frame_gray, (5,5),0) # Gaussian blur to remove noise

    # Adaptive threshold to find numbers on paper
    thresh = cv2.adaptiveThreshold(frame_gray,255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,35,7)
    thresh = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel,iterations=3)
    # cv2.imshow('thresh',thresh)

    contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(frame,contours,-1,(255,0,0),2)
    # Build the list of number contours and number locations

    if len(contours) < 35:
        for ind,contour in enumerate(contours):
            [x,y,w,h] = cv2.boundingRect(contour)
            if  bound < x < (frame_gray.shape[1] - bound) and bound < y < (frame_gray.shape[0] - bound) and (x+w) <= (frame_gray.shape[1] - bound) and (y+h) <= (frame_gray.shape[0] - bound):
                roi = frame_gray[y-bound:y+h+bound,x-bound:x+w+bound]

                if len(roi) > 0: # Gets rid of weird zero-size contours
                    new_roi = cv2.adaptiveThreshold(roi,255,
                        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,35,7)
                    new_roi = cv2.morphologyEx(new_roi,cv2.MORPH_OPEN,kernel,iterations=2) # Embiggen numbers
                    new_roi = cv2.resize(new_roi, (20,20), interpolation=cv2.INTER_AREA) # standardize contours
                    deskewed = deskew(new_roi)
                    # Record contour and contour location, and filter out internal contours
                    if hierarchy[0][ind][3] == -1:
                        cont = Character(deskewed,(x,y,w,h), hog(deskewed))
                        chars.append(cont)

    # Build an image to show all number contours
    num_len = len(chars)
    # print '# of contours: ', num_len
    if num_len < 35 and num_len > 0:
        new_img = np.ones((20,20*num_len),np.uint8)
        y = 0
        for x in chars:
            new_img[:,y:y+20] = x.img
            y += 20
        cv2.imshow('image3',new_img)
    return chars

    # Deskews a 20x20 character image
def deskew(img):
    SZ = 20
    affine_flags = cv2.WARP_INVERSE_MAP|cv2.INTER_LINEAR
    m = cv2.moments(img)
    if abs(m['mu02']) < 1e-2:
        return img.copy()
    # print m
    skew = m['mu11']/m['mu02']
    M = np.float32([[1,skew,-0.5*SZ*skew], [0,1,0]])
    img = cv2.warpAffine(img,M,(SZ,SZ),flags=affine_flags)
    return img

    # Retursn the HOG for a given imagej
def hog(img):
    bin_n = 16
    gx = cv2.Sobel(img, cv2.CV_32F,1,0) # x gradient
    gy = cv2.Sobel(img, cv2.CV_32F,0,1) # y gradient
    mag,ang = cv2.cartToPolar(gx,gy) # Polar gradients
    bins = np.int32(bin_n*ang/(2*np.pi)) # creating binvalues
    bin_cells = bins[:10,:10],bins[10:,:10],bins[:10,10:],bins[10:,10:]
    mag_cells = mag[:10,:10],mag[10:,:10],mag[:10,10:],mag[10:,10:]
    hists = [np.bincount(b.ravel(), m.ravel(), bin_n) for b, m in zip(bin_cells, mag_cells)]
    hist = np.hstack(hists) # hist is a 64-bit vector
    return hist
