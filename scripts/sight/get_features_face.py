import rospkg
import os
import cv2
import dlib

import dlib_features as dl
import numpy as np


class Features:
    def __init__(self, testing=False, samples=486, database='cohn-kanade'):
        # dlib face detector
        self.detector = dlib.get_frontal_face_detector()

        # defines file paths
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        if database == 'cohn-kanade':
            # path to face images
            self.faces_folder_path = PACKAGE_PATH + \
                '/params/database/cohn-kanade-images'
            # path to FACS labels
            self.FACS_labels_folder_path = PACKAGE_PATH + \
                '/params/database/FACS'
            # path to emotion labesl
            self.emotion_labels_folder_path = PACKAGE_PATH + \
                '/params/database/Emotion'

            self.face_detect = dl.FaceDetect()
            # list of file paths for face images
            self.file_paths = [os.path.join(root, file) for root, dir, files in
                               os.walk(self.faces_folder_path) for file in files]
            # list of file paths for FACS labels
            self.FACS_file_paths = [os.path.join(root, file) for root, dir, files in
                                    os.walk(self.FACS_labels_folder_path) for file in files]

        self.FACS = {'cheeks': [6, 11],
                     'mouth': [10, 11, 12, 13, 14, 15, 15, 17, 18, 20, 22, 23,
                               24, 25, 26, 27, 28],
                     'forehead': [1, 2, 4],
                     'nose': [9, 11],
                     'eyes': [5, 6, 7, 41, 42, 43, 44, 45, 46]}
        self.testing = testing
        self.samples = samples
        self.labels = dict()
        self.targets = []

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
        self.split = split
        return split

    def process_landmarks(self, clahe_image, face_region=None):
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
        for i, f in enumerate(self.file_paths[20:40]):
            clahe_images = self.process(f)
            region_landmarks = self.process_landmarks(clahe_images[0],
                                                      face_region)
            # append landmarks to features
            if region_landmarks != '':
                features.append(region_landmarks)
            region_landmarks_reflected = self.process_landmarks(clahe_images[1],
                                                                face_region)
            # append reflected landmarks to features
            if region_landmarks_reflected != '':
                features.append(region_landmarks_reflected)

        return np.asarray(features)

    def get_labels(self):
        for f in self.FACS_file_paths:
            # list of FACS labels for a given file
            FACS_list = []
            print(f)
            f_name = f[-36:-9]
            with open(f, 'r') as r:
                lines = r.readlines()
                for l in lines:
                    j = l.split(" ")
                    i = 0
                    while i < len(j):
                        elem = j[i]
                        elem = elem.strip()
                        if elem != "":
                            elem.strip()
                            val = float(elem)
                            i = len(j) + 10
                        else:
                            i += 1
                    FACS_list.append(val)
                self.labels[f_name] = FACS_list

    def get_labels_for_region(self, face_region):
        '''
        CHEEKS: 1-3, 14-16 (FACS 6, 11)
        MOUTH: 4-13, 48-68 (FACS 10-28 sans 19, 21)
        FOREHEAD: 17-30 (FACS 1, 2, 4)
        NOSE: 31-35 (FACS 9, 11)
        EYES: 36-47 (FACS 5-7, 41-46)
        '''
        # dictionary of keys face region and values FACS for that region
        self.get_labels()
        labels_region = dict()
        for f in self.labels.keys():
            for FAC in self.FACS[face_region]:
                if FAC in self.labels[f]:
                    labels_region[f] = FAC
                else:
                    labels_region[f] = 0.0
        return labels_region

    def get_targets(self, face_region):
        labels_region = self.get_labels_for_region(face_region='nose')
        # get targets for face region_
        for f in self.file_paths:
            f_name = f[-31:-4]
            # print(f_name)
            # print(labels_region[f_name])
            if f_name in self.labels.keys():
                print('yass', labels_region[f_name])
                self.targets.append(labels_region[f_name])
        # if no label, make target = -1
            else:
                self.targets.append(-1)

        self.targets = np.asarray(self.targets)
        return self.targets


if __name__ == "__main__":
    f = Features()
    # f.get_labels()
    # features = f.get_features(face_region='nose')
    targets = f.get_targets(face_region='nose')
    print(targets)
