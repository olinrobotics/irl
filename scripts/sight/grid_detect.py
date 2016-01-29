import cv2
import numpy as np
from matplotlib import pyplot as plt
import operator
import math

def get_distance(pt1, pt2):
	return math.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

img = cv2.imread("test_imgs/2.jpg")
h, w, ch = img.shape

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

corners = cv2.goodFeaturesToTrack(gray,25,0.01,70)
corners = np.int0(corners)

dists = {}
for num, i in enumerate(corners):
    x,y = i.ravel()
    dists[num] = math.sqrt((x-(w/2))**2 + (y-(h/2))**2)

sorted_dists = sorted(dists.items(), key=operator.itemgetter(1))
pts = []
for i in range(4):
	pts.append(sorted_dists[i])

# try:
pt1 = corners[pts[0][0]].ravel()
pt2 = corners[pts[1][0]].ravel()
pt3 = corners[pts[2][0]].ravel()
pt4 = corners[pts[3][0]].ravel()

pt_dists = []
pt_dists.append((1, get_distance(pt1, pt2)))
pt_dists.append((2, get_distance(pt1, pt3)))
pt_dists.append((3, get_distance(pt1, pt4)))
pt_dists.append((4, get_distance(pt2, pt3)))
pt_dists.append((5, get_distance(pt2, pt4)))
pt_dists.append((6, get_distance(pt3, pt4)))

pt_dists = sorted(pt_dists, key=lambda x: x[1])
print pt_dists

cv2.line(img, tuple(pt1), tuple(pt2), (0,255,0))
cv2.line(img, tuple(pt1), tuple(pt3), (0,255,0))
cv2.line(img, tuple(pt2), tuple(pt4), (0,255,0))
cv2.line(img, tuple(pt3), tuple(pt4), (0,255,0))

# except Exception, e:
# 	print "err"
# 	pass

	# x,y = corners[point[0]].ravel()
	# cv2.circle(img,(x,y),3,255,-1)


cv2.imshow("im", img)
c = cv2.waitKey(0)

cv2.destroyAllWindows()


# gray = np.float32(gray)
# dst = cv2.cornerHarris(gray,2,3,0.04)

# #result is dilated for marking the corners, not important
# dst = cv2.dilate(dst,None)
# points = dst.getHarrisPoints()

# # Threshold for an optimal value, it may vary depending on the image.
# img[dst>0.01*dst.max()]=[0,0,255]

# cv2.imshow('dst', img)
# if cv2.waitKey(0) & 0xff == 27:
#     cv2.destroyAllWindows()

# imgSplit = cv2.split(im)
# flag,b = cv2.threshold(imgSplit[2],0,255,cv2.THRESH_OTSU)

# element = cv2.getStructuringElement(cv2.MORPH_CROSS,(1,1))
# cv2.erode(b,element)

# edges = cv2.Canny(b,150,200,3,5)
# img = im.copy()

# lines = cv2.HoughLinesP(edges,1,np.pi/2,2, minLineLength = 200, maxLineGap = 100)[0]

# for x1,y1,x2,y2 in lines:
# 	for index, (x3,y3,x4,y4) in enumerate(lines):

# 		if y1==y2 and y3==y4: # Horizontal Lines
# 			diff = abs(y1-y3)
# 		elif x1==x2 and x3==x4: # Vertical Lines
# 			diff = abs(x1-x3)
# 		else:
# 			diff = 0

# 		if diff < 10 and diff is not 0:
# 			del lines[index]

# gridsize = (len(lines) - 2) / 2

# for x1,y1,x2,y2 in lines:
# 	cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)

# cv2.imshow('houghlines',img)
# c = cv2.waitKey(0)

# cv2.destroyAllWindows()