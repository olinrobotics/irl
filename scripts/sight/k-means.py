# ref http://scikit-learn.org/stable/auto_examples/cluster/plot_kmeans_assumptions.html#sphx-glr-auto-examples-cluster-plot-kmeans-assumptions-py
# http://scikit-learn.org/stable/auto_examples/cluster/plot_kmeans_digits.html
from sklearn import metrics
from sklearn.cluster import KMeans
from sklearn.datasets import PCA
from sklean.decompoisition import scale


# Default data_set from sklean.datasets: http://scikit-learn.org/stable/modules/classes.html#module-sklearn.datasets
# The data_set is a tuple (X, y) where X = an n_samples by n_features numpy array
# and an array of length n_samples
class Cluster:
    def __init__(self, data_set=load_digits()):
        self.data_set = data_set
        self.data = scale(self.data_set.data)
        self.n_samples, self.n_features = self.data.shape
        self.n_clusters = len(np.unique(digits.target))
        self.sample_size = 300

    def bench_k_means(estimator, name, data):
        t0 = time()
        estimator.fit(self.data)
        

if __name__ == "__main__":
