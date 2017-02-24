from skimage import data
import numpy as np

# grab some sample data
camera = data.camera()

# MASKING
# create mask for camera with values greater than 87 masked
mask = camera > 123
# set mask to white (255) pixels where mask is True
camera[mask] = 255
# makes an array the same size of the image
nrows, ncols = camera.shape
row, col = np.ogrid[:nrows, :ncols]
# can also use boolean artihmetic for more complex masks (see np.logical_and)


# FANCY INDEXING
# values from 0 to number of rows
ind_r = np.arange(len(camera))
# every fourth column (this workds b/c rows = columns)
ind_c = 4 * ind_r % len(camera)
camera[ind_r, ind_c] = 0

# COLOR IMAGES
# color images will have a depth of 3
cat = data.chelsea()
print(cat.shape)

# using masks in color images
reddish = cat[:, :, 0] > 160
cat[reddish] = [0, 255, 0]

# look for coordinate conventions and array order notes:
# http://scikit-image.org/docs/dev/user_guide/numpy_images.html

# TODO: load an image as a numpy array
