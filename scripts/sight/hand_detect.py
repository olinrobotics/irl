import numpy as np
import cv2


def main():
    cap = cv2.VideoCapture(0)
    first_frame = None
    i = 0

    while True:
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)[1]

        # if i == 0:
        #     first_frame = thresh
        #     i += 1
        #     continue

        # while i < 20:
        #     first_frame +=
        #     first_frame = first_frame/i
        #     i += 1
        #     continue
        if first_frame == None:
            print "assigned first frame"
            first_frame = thresh
            continue

        img = thresh - first_frame
        cv2.imshow("img", img)
        c = cv2.waitKey(1)

if __name__ == "__main__":
    main()