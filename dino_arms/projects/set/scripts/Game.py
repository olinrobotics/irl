from Turn import *
from opencv import *
import argparse
import cv2

"""
Authors: Cassandra Overney and Enmo Ren
Purpose: Integrates the opencv, Set and Turn files. Basically takes an image and returns a set from it with the cards coordinates

"""

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="path to the image file")
args = vars(ap.parse_args())
image = cv2.imread("test106.jpg")
print image
results = find_matches(image)
#print("final", results)

turn1 = Turn(results)
result = turn1.find_set()
turn1.print_card_array(turn1.card_array)
cv2.imshow('Image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()