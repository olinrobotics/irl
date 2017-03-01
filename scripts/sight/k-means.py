# ref http://scikit-learn.org/stable/auto_examples/cluster/plot_kmeans_assumptions.html#sphx-glr-auto-examples-cluster-plot-kmeans-assumptions-py
# http://scikit-learn.org/stable/auto_examples/cluster/plot_kmeans_digits.html
from sklearn import metrics
from sklearn.cluster import KMeans
from sklearn.datasets import PCA
from sklean.decompoisition import scale


class Cluster:
    def __init__(self, data=scale(load_digits().data)):
        self.data = data
        self.n_samples, self.n_features = self.data.shape


if __name__ == "__main__":
