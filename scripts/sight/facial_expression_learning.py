import rospkg
import os
import numpy as np

from sklearn.semi_supervised import LabelPropagation
from sklearn.externals import joblib

import get_features_face as gf


class FACSTrainer:
    def __init__(self, face_region, samples=486):
        # File paths
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.params_path = PACKAGE_PATH + '/params'
        # face region and features object
        self.face_region = face_region
        self.features = gf.Features(samples=samples)
        # get training data from database using get_features_face script
        self.train_data_set = self.features.get_features(face_region=face_region)
        print(self.train_data_set.shape)
        # targets labeled at FAC Unit, 0 for neutral, or -1 for unlabeled
        self.targets = self.features.get_targets(face_region=face_region)
        print(self.targets)
        print(self.targets.shape)
        # n_clusters - depends on face_region, number of FACS in the region
        # plus one for neutral
        # FAC Unit 4 on the forehead is broken into 3 parts: left, right, both
        # FAC Unit 12 on the mouth is broken into 3 parts: left, right, both
        # FAC Unit 46 on eyes is broken into 3 parts: left, rightn, both
        n_clusters = {'cheeks': 3,
                      'mouth': 20,
                      'forehead': 6,
                      'nose': 3,
                      'eyes': 12}
        self.n_clusters = n_clusters[face_region]

        classif_file_path = self.params_path + face_region + 'pkl'

        # if os._exists(classif_file_path):
        #     self.classifier = joblib.load(classif_file_path)
        # else:
        self.classifier = LabelPropagation()

    def fit_data(self):
        self.classifier.fit(X=self.train_data_set, y=self.targets)

    def save_classifier(self):
        joblib.dump(self.classifier, self.params_path + self.face_region + '.pkl')

# class FACSPredictor


if __name__ == "__main__":
    # features = gf.Features()
    # test_data = features.get_test_data(face_region='nose')
    # nose_trainer = joblib.load('nose.pkl')

    # nose_trainer.find_labels()

    nose_trainer = FACSTrainer(face_region='nose')
    nose_trainer.fit_data()
    nose_trainer.save_classifier()
    # nose_trainer.save_classifier
