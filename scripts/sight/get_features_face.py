import rospkg
import os
import cv2
import dlib

import dlib_features as dl
import numpy as np


class Features:
    def __init__(self, testing=False, samples=70, database='cohn-kanade'):
        # dlib face detector
        self.detector = dlib.get_frontal_face_detector()

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

    def read_file(self, f):
        # read file at file path f
        im = cv2.imread(f)
        # convert to grayscale
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        return gray

    def crop_to_face(self, gray):
        # CROP TO FACE
        # Find faces using opencv
        faces = self.detector(gray, 1)

        biggest_area = 0

        for face in faces:
            x = face.left()
            y = face.top()
            w = face.right() - x
            h = face.bottom() - y
            area = w*h
            if area > biggest_area:
                face = gray[y:y+h, x:x+w]
                biggest_area = area

        return face

    def get_clahe_image(self, face):
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        clahe_image = clahe.apply(face)
        return clahe_image

    def process(self, f):
        ''''
        - reads file
        - will crop image to face only
        - returns processed clahe_image, return type will be a tuple of original
        clahe_image and reflected clahe_image'''
        gray = self.read_file(f)
        face = self.crop_to_face(gray)
        clahe_image = self.get_clahe_image(face)

        face_copy = face.copy()
        reflected_face = cv2.flip(face_copy, 0)
        reflected_clahe_image = self.get_clahe_image(reflected_face)

        return clahe_image, reflected_clahe_image

    def split_landmarks(self, landmarks):
        '''
        splits landmarks (givin in list within list) into dictionary
        keys: 'cheeks', 'mouth', 'forhead', 'nose', 'eyes'

        CHEEKS: 1-3, 14-16 (FACS 6, 11)
        MOUTH: 4-13, 48-68 (FACS 10-28 sans 19, 21)
        FOREHEAD: 17-30 (FACS 1, 2, 4)
        NOSE: 31-35 (FACS 9, 11)
        EYES: 36-47 (FACS 5-7, 41-46)
        '''
        split = dict()
        split['cheeks'] = landmarks[0:2] + landmarks[13:15]
        split['mouth'] = landmarks[3:12] + landmarks[47:67]
        split['forehead'] = landmarks[16:29]
        split['nose'] = landmarks[30:34]
        split['eyes'] = landmarks[4:6] + landmarks[40:45]
        return split

    def process_landmarks(self, clahe_image, face_region):
        landmarks = self.face_detect.get_landmarks(clahe_image,
                                                   return_type='nparray')
        split = self.split_landmarks(landmarks)
        region_landmarks = split[face_region]
        return region_landmarks

    def get_features(self, face_region=None):
        # Generator statement: generates lists of file paths
        features = []
        for i, f in enumerate(self.file_paths[:self.samples]):
            clahe_images = self.process(f)
            region_landmarks = self.process_landmarks(clahe_images[0],
                                                      face_region)
            # append landmarks to Features
            if region_landmarks != '':
                features.append(region_landmarks)
            region_landmarks_reflected = self.process_landmarks(clahe_images[1],
                                                                face_region)
            # append landmarks to Features
            if region_landmarks_reflected != '':
                features.append(region_landmarks_reflected)

        return np.asarray(features)

    def get_test_data(self, face_region=None):
        # Generator statement: generates lists of file paths
        features = []
        for i, f in enumerate(self.file_paths[-20:]):
            print(f)
            clahe_images = self.process(f)
            region_landmarks = self.process_landmarks(clahe_images[0],
                                                      face_region)
            # append landmarks to Features
            if region_landmarks != '':
                features.append(region_landmarks)
            region_landmarks_reflected = self.process_landmarks(clahe_images[1],
                                                                face_region)
            # append landmarks to Features
            if region_landmarks_reflected != '':
                features.append(region_landmarks_reflected)

        return np.asarray(features)


if __name__ == "__main__":
    f = Features()
    features = f.get_features(face_region='nose')
    print(features)
