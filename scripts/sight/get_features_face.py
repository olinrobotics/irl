import rospkg
import rospy
import os
import cv2

import dlib_ex as dl


class Features:
    def __init__(self):
        # defines file paths
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.faces_folder_path = PACKAGE_PATH + '/params/cohn-kanade-images'
        self.emotion_labels_folder_path = PACKAGE_PATH + '/params/Emotion'

        self.face_detect = dl.FaceDetect()

    def get_clahe_image(self, f):
        im = cv2.IMREAD_GRAYSCALE(f)
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        clahe_image = clahe.apply(gray)
        return clahe_image

    def get_features(self):
        fd = dl.FaceDetect()
        for path, dirname, filename in os.walk(self.faces_folder_path):
            for f in filename:
                clahe_image = self.get_clahe_image(f)
                print(fd.get_landmarks(clahe_image))


if __name__ == "__main__":
    f = Features()
    f.get_features()
