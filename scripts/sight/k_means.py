
# ref http://scikit-learn.org/stable/auto_examples/cluster/plot_kmeans_assumptions.html#sphx-glr-auto-examples-cluster-plot-kmeans-assumptions-py
# http://scikit-learn.org/stable/auto_examples/cluster/plot_kmeans_digits.html
from time import time
import numpy as np

from sklearn import metrics
from sklearn.cluster import KMeans
from sklearn.datasets import load_digits
from sklearn.preprocessing import scale


# Default data_set from sklean.datasets: http://scikit-learn.org/stable/modules/classes.html#module-sklearn.datasets
# The data_set is a tuple (X, y) where X = an n_samples by n_features numpy array
# and an array of length n_samples
class Clusters():
    def __init__(self, data_set=load_digits(), n_clusters=5, n_init=1,
                 init='k-means++'):
        self.data_set = data_set
        self.data = scale(self.data_set.data)
        self.n_samples, self.n_features = self.data.shape
        self.n_clusters = n_clusters
        self.labels = self.data_set.target
        self.sample_size = 300
        self.n_init = n_init
        self.init = init
        self.estimator = KMeans(init=self.init, n_clusters=self.n_clusters,
                                n_init=self.n_init)
        # Fit data
        self.estimator.fit(self.data)

    def bench_k_means(self):
        t0 = time()
        print('% 9s   %.2fs    %i   %.3f   %.3f   %.3f   %.3f   %.3f    %.3f'
              % (self.init, (time() - t0), self.estimator.inertia_,
                 metrics.homogeneity_score(self.labels, self.estimator.labels_),
                 metrics.completeness_score(self.labels, self.estimator.labels_),
                 metrics.v_measure_score(self.labels, self.estimator.labels_),
                 metrics.adjusted_rand_score(self.labels, self.estimator.labels_),
                 metrics.adjusted_mutual_info_score(self.labels, self.estimator.labels_),
                 metrics.silhouette_score(self.data, self.estimator.labels_,
                                          metric='euclidean',
                                          sample_size=self.sample_size)))

    # Return cluster centers
    def get_cluster_centers(self):
        return self.estimator.cluster_centers_

    def get_estimator(self):
        return self.estimator

    # return classification goodness metric
    def check_accuracy(self, test_data):
        y_pred = self.estimator.predict(test_data)
        return(y_pred)
        # score = metrics.accuracy_score(y_true=y_true, y_pred=y_pred)
        # return score

    def print_data_set(self):
        print(self.data_set)


if __name__ == "__main__":
    clusters_kmeans = Clusters()
    clusters_kmeans.bench_k_means()
    clusters_random = Clusters(init='random')
    clusters_random.bench_k_means()
    # print(clusters_kmeans.get_cluster_centers())
    # print(clusters_kmeans.check_accuracy(clusters_kmeans.data))
    clusters_kmeans.print_data_set()
