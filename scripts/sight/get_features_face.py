import rospkg
import rospy
import os
import cv2

import dlib_features as dl
import numpy as np


class Features:
    def __init__(self, testing=True, samples=2, database='cohn-kanade'):
        # defines file paths
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        if database == 'cohn-kanade':
            self.faces_folder_path = PACKAGE_PATH + \
                '/params/database/cohn-kanade-images'
            self.emotion_labels_folder_path = PACKAGE_PATH + '/params/Emotion'
            self.face_detect = dl.FaceDetect()
            self.file_paths = [os.path.join(root, file) for root, dir, files in
                               os.walk(self.faces_folder_path) for file in files]
        self.testing = testing
        self.samples = samples

    def get_clahe_image(self, f):
        # file_path = self.faces_folder_path + '/' + f
        # print(file_path)
        im = cv2.imread(f)
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        clahe_image = clahe.apply(gray)
        return clahe_image

    def split_landmarks(self, landmarks):
        '''
        splits landmarks (givin in list within list) into dictionary
        keys: 'cheeks', 'mouth', 'forhead', 'nose', 'eyes'

        CHEEKS: 1-3, 14-16 (FACS 6)
        MOUTH: 4-13, 48-68 (FACS 10-28 sans 19, 21)
        FOREHEAD: 17-30 (FACS 1, 2, 4)
        NOSE: 31-35 (FACS 9)
        EYES: 36-47 (FACS 5-7, 41-46)
        '''
        split = dict()
        split['cheeks'] = landmarks[0:2] + landmarks[13:15]
        split['mouth'] = landmarks[3:12] + landmarks[47:67]
        split['forehead'] = landmarks[16:29]
        split['nose'] = landmarks[30:34]
        split['eyes'] = landmarks[4:6] + landmarks[40:45]
        return split

    def get_features(self, face_region=None):
        # Generator statement: generates lists of file paths
        # TODO: Make Features array
        # TODO: Crop
        # TODO: Reflect and duplicate
        features = []
        print(self.file_paths[0])
        if self.testing:
            clahe_image = self.get_clahe_image(self.file_paths[0])
            landmarks = self.face_detect.get_landmarks(clahe_image, return_type='nparray')
            return(landmarks)
        else:
            for i, f in enumerate(self.file_paths[:self.samples]):
                clahe_image = self.get_clahe_image(f)
                landmarks = self.face_detect.get_landmarks(clahe_image, return_type='nparray')
                split = self.split_landmarks(landmarks)
                region_landmarks = split[face_region]
                # append landmarks to Features
                features.append(region_landmarks)
            return np.asarray(features)
            # TODO: reflect the face and get landmarks
            # TODO: crop image


if __name__ == "__main__":
    f = Features(testing=False)
    features = f.get_features(face_region='nose')
    print(features)
