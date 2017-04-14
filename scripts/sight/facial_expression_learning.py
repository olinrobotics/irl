import rospkg
import os

from sklearn.cluster import KMeans
from sklearn.externals import joblib

import get_features_face as gf


class FACSTrainer:
    def __init__(self, face_region, samples=486, n_init=5):
        # File paths
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.params_path = PACKAGE_PATH + '/params'
        # face region and features object
        self.face_region = face_region
        self.features = gf.Features(samples=samples)
        # get training data from database using get_features_face script
        self.train_data_set = self.features.get_features(face_region=face_region)
        self.test_data_set = self.features.get_test_data(face_region=face_region)
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

        if os._exists(classif_file_path):
            self.classifier = joblib.load(classif_file_path)
        else:
            self.classifier = KMeans(init='k-means++', n_clusters=self.n_clusters,
                                     n_init=n_init)

    def fit_data(self):
        self.classifier.fit(self.train_data_set, y=None)

    def save_classifier(self):
        joblib.dump(self.classifier, self.params_path + self.face_region + '.pkl')

    def find_labels(self):
        f_names = self.features.file_paths
        clusters = self.classifier.predict(self.train_data_set)
        labels = self.features.labels
        # dictionary with keys: f_names, values: clusters
        file_clusters = dict()
        for f, c in zip(f_names, clusters):
            file_clusters[f] = c

        # loop through labels
        for key in labels.keys():
            f_name = key[:-3]
            print(f_name)


# class FACSPredictor


if __name__ == "__main__":
    # features = gf.Features()
    # test_data = features.get_test_data(face_region='nose')
    # nose_trainer = joblib.load('nose.pkl')

    # nose_trainer.find_labels()

    nose_trainer = FACSTrainer(face_region='nose')
    # nose_trainer.fit_data()
    # nose_trainer.save_classifier
    print(nose_trainer.classifier)
