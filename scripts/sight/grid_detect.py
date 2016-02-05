import cv2
import numpy as np
from matplotlib import pyplot as plt
import operator
import math
import itertools

def collinear(p0, p1, p2):
    x1, y1 = p1[0] - p0[0], p1[1] - p0[1]
    x2, y2 = p2[0] - p0[0], p2[1] - p0[1]
    return x1 * y2 - x2 * y1 < 1e-12

def get_distance(pt1, pt2):
	return math.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

def get_pt_x(pt1, pt2, d):
	v = (pt1[0]-pt2[0], pt1[1]-pt2[1])
	v_mag = get_distance(pt1, pt2)

	u = (v[0]/v_mag, v[1]/v_mag)

	return (int(pt1[0]+d*u[0]), int(pt1[1]+d*u[1]))

def get_box(ref_box, h, w):
	pt0 = ref_box[0]
	pt1 = ref_box[1]
	pt2 = get_pt_x(ref_box[1], ref_box[2], h)
	pt3 = get_pt_x(ref_box[0], ref_box[3], h)

	return [pt0, pt1, pt2, pt3]

def get_grid(box):
	grid_height = int(get_distance(box[0], box[1]))
	grid_width = int(get_distance(box[1], box[3]))

	box2 = get_box([box[2], box[1], box[0], box[3]], grid_height, grid_width)
	box4 = get_box(box, grid_height, grid_width)
	box5 = get_box([box[3], box[2], box[1], box[0]], grid_height, grid_width)
	box7 = get_box([box[3], box[0], box[1], box[2]], grid_height, grid_width)

	box1 = get_box([box4[2], box4[1], box4[0], box4[3]], grid_height, grid_width)
	box6 = get_box([box4[3], box4[0], box4[1], box4[2]], grid_height, grid_width)

	box3 = get_box([box5[2], box5[1], box5[0], box5[3]], grid_height, grid_width)
	box8 = get_box([box5[3], box5[0], box5[1], box5[2]], grid_height, grid_width)

	return [box1, box2, box3, box4, box5, box6, box7, box8]

def get_center_box(im_in):
	# img = cv2.imread(im_in)
	img = im_in
	h, w, ch = img.shape

	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	corners = cv2.goodFeaturesToTrack(gray,25,0.01,120)
	corners = np.int0(corners)

	dists = {}
	for num, i in enumerate(corners):
		x,y = i.ravel()
		cv2.circle(img,(x,y),3,255,-1)
		dists[num] = math.sqrt((x-(w/2))**2 + (y-(h/2))**2)

	sorted_dists = sorted(dists.items(), key=operator.itemgetter(1))
	center_rect = []

	pts_distances = []

	for i in range(4):
		pts_distances.append(sorted_dists[i])
		center_rect.append(list(corners[pts_distances[i][0]].ravel()))

	print center_rect

	rect = cv2.minAreaRect(np.int0(center_rect))

	box = cv2.cv.BoxPoints(rect)
	box = np.int0(box)

	boxes = get_grid(box)

	for box in boxes:
		box = np.int0(box)
		cv2.drawContours(img,[box],0,(0,0,255),2)

	return img

def main():
	cap = cv2.VideoCapture(1)
	while True:
		ret, frame = cap.read()
		img = get_center_box(frame)
		cv2.imshow("img", frame)
		c = cv2.waitKey(1)

if __name__ == "__main__":
	main()