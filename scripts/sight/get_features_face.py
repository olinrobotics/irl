import rospkg
import rospy
import os
import cv2

import dlib_ex as dl


class Features:
    def __init__(self, testing=True):
        # defines file paths
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.faces_folder_path = PACKAGE_PATH + \
            '/params/database/cohn-kanade-images'
        self.emotion_labels_folder_path = PACKAGE_PATH + '/params/Emotion'
        self.face_detect = dl.FaceDetect()
        self.file_paths = [os.path.join(root, file) for root, dir, files in
                           os.walk(self.faces_folder_path) for file in files]
        self.testing = testing

    def get_clahe_image(self, f):
        # file_path = self.faces_folder_path + '/' + f
        # print(file_path)
        im = cv2.imread(f)
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        clahe_image = clahe.apply(gray)
        return clahe_image

    def get_features(self):
        # Generator statement: generates lists of file paths
        # TODO: Make Features array
        if self.testing:
            clahe_image = self.get_clahe_image(self.file_paths[0])
            landmarks = self.face_detect.get_landmarks(clahe_image, return_type='nparray')
            return(landmarks)
        else:
            for f in self.file_paths:
                # print(f)
                clahe_image = self.get_clahe_image(f)
                landmarks = self.face_detect.get_landmarks(clahe_image, return_type='nparray')
                # TODO: append landmarks to Features


if __name__ == "__main__":
    f = Features()
    f.get_features()
