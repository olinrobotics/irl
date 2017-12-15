from Turn import *
from opencv import *
import numpy as np
import matplotlib.pyplot as plt
import argparse
import cv2

"""
Authors: Cassandra Overney and Enmo Ren
Purpose: Integrates 
- What does this script do
- What dependencies do i need
- How do i run this script (it might just be python irl_template.py, or it might run by
something else that imports this script)
"""

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help="path to the image file")
args = vars(ap.parse_args())
image = cv2.imread(args["image"])
results = find_matches(image)
print("final", results)

turn1 = Turn(results)
result = turn1.find_set()
turn1.print_card_array(result)