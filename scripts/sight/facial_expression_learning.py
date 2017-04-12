from sklearn.cluster import KMeans
from sklearn.externals import joblib

import get_features_face as gf


class FACSTrainer:
    def __init__(self, face_region, samples=486, n_init=5):
        self.face_region = face_region
        self.features = gf.Features(samples=samples)
        # get training data from database using get_features_face script
        self.train_data_set = self.features.get_features(face_region=face_region)
        self.test_data_set = self.features.get_test_data(face_region=face_region)
        # n_clusters - depends on face_region, number of FACS in the region
        # plus one for neutral
        # FAC Unit 4 on the forehead is broken into 3 parts: left, right, both
        # FAC Unit 12 on the mouth is broken into 3 parts: left, right, both
        # FAC Unit 46 on eyes is broken into 3 parts: left, right, both
        n_clusters = {'cheeks': 3,
                      'mouth': 20,
                      'forehead': 6,
                      'nose': 3,
                      'eyes': 12}
        self.n_clusters = n_clusters[face_region]
        self.classifier = KMeans(init='k-means++', n_clusters=self.n_clusters,
                                 n_init=n_init)

    def fit_data(self):
        self.classifier.fit(self.train_data_set, y=None)

    def save_classifier(self):
        joblib.dump(self.classifier, self.face_region + '.pkl')

# class FACSPredictor


if __name__ == "__main__":
    features = gf.Features()
    test_data = features.get_test_data(face_region='nose')
    nose_trainer = joblib.load('nose.pkl')

    print(nose_trainer.predict(test_data))
