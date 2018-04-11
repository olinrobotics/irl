import cv2
import numpy as np

image = cv2.imread('image_4.jpg')
rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
lower_white = np.array([120,120,120])
upper_white = np.array([255,255,255])

mask = cv2.inRange(rgb, lower_white, upper_white)

# The bitwise and of the frame and mask is done so
# that only the white coloured objects are highlighted
# and stored in res
res = cv2.bitwise_and(image,image, mask=mask)
cv2.imshow('image', image)
cv2.imshow('mask', mask)
cv2.imshow('res', res)

cv2.waitKey(0)
