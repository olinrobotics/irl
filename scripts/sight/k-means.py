# code from http://scikit-learn.org/stable/auto_examples/cluster/plot_kmeans_assumptions.html#sphx-glr-auto-examples-cluster-plot-kmeans-assumptions-py

from sklearn.cluster import KMeans
from sklearn.datasets import make_blobs

import matplotlib.pyplot as plt
import numpy as np

plt.figure(figsize=(12, 12))

n_samples = 1500
random_state = 170
X, y = make_blobs(n_samples=n_samples, random_state=random_state)

# Incorrect number of clusters
y_pred = KMeans(n_clusters=2, random_state=random_state)

# Makes a scatter plot of x vs. y
plt.subplot(221)
plt.scatter(X[:, 0], X[:, 1],  c=y_pred)
plt.title("Incorrect Number of Blobs")

# Anisotropicly distributed data (not distributed symetrically)
transformation = [[0.60834549, -0.63667341], [-0.40887718, 0.85253229]]

# X = np.array([[1, 1], [1, 2], [2, 1], [2, 2], [5, 5], [5, 6], [6, 5], [6, 6],
              # [7, 1], [7, 2], [8, 1], [8, 2]])
# .fix computes k-means clustering
# '''kmeans = KMeans(n_clusters=3, random_state=0).fit(X)
# estimator = KMeans(n_clusters=3)

# samples = 12
# features = 3
# labels = estimator.labels_
# # kmeans.predict(X)

# # Ploting figure
# plt.clf
# fig = plt.figure(figsize=(4, 3))
# plt.cla
# plt.plot(X)
# plt.show()'''
