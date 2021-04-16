import pdb

import cv2
import numpy as np
from matplotlib import pyplot as plt

path = "/home/rxth/catkin_ws/src/CameraProjectorProjection/camera_projector/data/images/"
# filename = "tree_dpv_depth_single.png"
# filename = "tree_dpv_depth_single2.png"
filename = "tree_dpv_depth_multi.png"
# filename = "tree_dpv_depth_multi2.png"
# filename = "tree_dpv_depth_multi3.png"

# img = plt.imread(path+filename, "mono8")
# img = plt.imread(path+filename, "mono16")
# img = plt.imread(path+filename)
img = cv2.imread(path+filename)
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# plt.imshow(img, cmap="gray")
plt.imshow(img, cmap="gray", vmin=0, vmax=255)
plt.show()

# img = np.array(img)
# img_mask = np.zeros(np.shape(img))
img_mask = np.zeros(np.shape(img), dtype="uint8")
print(img<255)
# img_mask[img<255] = 200
img_mask[img<15] = 255
# img_mask[img<25] = 255
# img_mask[(img<255)] = 255
# plt.imshow(img_mask, cmap="gray")
plt.imshow(img_mask, cmap="gray", vmin=0, vmax=255)
plt.show()

