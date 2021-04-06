import pdb

import cv2
import copy
import numpy as np
from matplotlib import pyplot as plt

path = "/home/rxth/catkin_ws/src/CameraProjectorProjection/camera_projector/data/images/"
filename = "img_from_topic.png"

img = plt.imread(path+filename, "bgr8") # If format to read it in is not mentioned, then pixels are in 0-1 range.
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# plt.imshow(img_gray, cmap="gray")
# plt.show()


# Trying to Improve the contrast of the image...incase the corners can't be detected.
# But that is not needed.
'''
hist, bins = np.histogram(img_gray.flatten(), 256, [0,256])
cdf = hist.cumsum()
cdf_normalized = cdf * hist.max()/ cdf.max()

plt.plot(cdf_normalized, color = 'b')
plt.hist(img_gray.flatten(),256,[0,256], color = 'r')
plt.xlim([0,256])
plt.legend(('cdf','histogram'), loc = 'upper left')
plt.show()
img_gray_histEq = cv2.equalizeHist(img_gray)
plt.imshow("img", img_gray_histEq)
plt.show()
'''

nrow = 7
ncol = 9
pattern_dims = (nrow-1, ncol-1)
ret, corners = cv2.findChessboardCorners(img_gray, pattern_dims, None)
img_corners = copy.deepcopy(img)
if(ret):
    print(corners)
    cv2.drawChessboardCorners(img_corners, pattern_dims, corners, ret)
else:
    print("No corners found.")
plt.imshow(img_corners)
plt.show()


sq_len = 0.0185 # 18.5mm
K = np.array( [[385.2106018066406, 0.0, 318.61053466796875], [0.0, 385.2106018066406, 242.80691528320312], [0.0, 0.0, 1.0]] )
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [[385.2106018066406, 0.0, 318.61053466796875], [0.0, 0.0, 385.2106018066406], [242.80691528320312, 0.0, 0.0, 0.0, 1.0, 0.0]]

q = corners[47][0]
q = np.array(q)
q = np.transpose(q)
Q = [[0], [nrow*sq_len]]
Q = np.array(Q)

# q = s*M*[R|t]*Q
# homog = np.linalg.inv(K)*q*np.linalg.inv(Q) # inverse of a vector Q can't be taken/has no meaning. Need to solve as a system of linear eqns instead!!

print("===")
obj_pts = []
for i in range(ncol-1-1, 0-1, -1): # rows
    for j in range(0, nrow-1, +1): # cols
        # print([i, j, 0])
        obj_pts.append([i*sq_len, j*sq_len, 0])
obj_pts = np.array(obj_pts)        
print(obj_pts)


# objpoints = obj_pts
# imgpoints = corners
# ret_, mtx_, dist_, rvecs_, tvecs_ = cv2.calibrateCamera(objpoints, imgpoints, img_gray.shape[::-1], None, None)

# ret,rvecs, tvecs = cv.solvePnP(objp, corners2, mtx, dist)
# ret,rvecs, tvecs = cv2.solvePnP(obj_pts, corners, K, [0,0,0,0,0])
# ret,rvecs, tvecs = cv2.solvePnP(obj_pts, corners, K)
# cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.CV_ITERATIVE)
corners = np.array(corners)
dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
# ret,rvecs, tvecs = cv2.solvePnP(obj_pts, corners, K, dist_coeffs, flags=cv2.CV_ITERATIVE)
pdb.set_trace()
ret,rvecs, tvecs = cv2.solvePnP(obj_pts, corners, K, dist_coeffs, flags=cv2.cv2.SOLVEPNP_ITERATIVE)

print("===")
print(rvecs)
print(tvecs)












# https://answers.opencv.org/question/209890/camera-pose-from-checkerboard-pose/
# https://answers.opencv.org/question/209890/camera-pose-from-checkerboard-pose/
# https://answers.opencv.org/question/209890/camera-pose-from-checkerboard-pose/
# https://answers.opencv.org/question/209890/camera-pose-from-checkerboard-pose/

# https://docs.opencv.org/3.4/d7/d53/tutorial_py_pose.html
# https://docs.opencv.org/3.4/d7/d53/tutorial_py_pose.html
# https://docs.opencv.org/3.4/d7/d53/tutorial_py_pose.html
# https://docs.opencv.org/3.4/d7/d53/tutorial_py_pose.html






'''
*check if saved witout bgr8 what happens

'''