import pickle

from sklearn.cluster import KMeans

import get_features_face as gf


class EmotionTrainer:
    def __init__(self, face_region, samples=20, n_init=5):
        self.features = gf.Features(samples=samples)
        # get training data from database using get_features_face script
        self.train_data_set = self.features.get_features(face_region=face_region)
        # n_clusters - depends on face_region
        self.classifier = KMeans(init='k-means++', n_clusters=self.n_clusters,
                                 n_init=n_init)
