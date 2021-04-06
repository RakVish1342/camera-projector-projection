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

nrows = 7
ncols = 9
pattern_dims = (nrows-1, ncols-1)
ret, corners = cv2.findChessboardCorners(img_gray, pattern_dims, None)
img_corners = copy.deepcopy(img)
if(ret):
    print(corners)
    # Optional refining of corners in 10x10 pixel vicinity
    # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # corners = cv2.cornerSubPix(img_corners, corners, (10, 10), (-1,-1), criteria)
    cv2.drawChessboardCorners(img_corners, pattern_dims, corners, ret)
else:
    print("No corners found.")
plt.imshow(img_corners)
plt.show()

# From ROS topic and physical sheet
sq_len = 0.0185 # 18.5mm
D = np.zeros((4,1)) # No lens distortion as per ros topic
K = np.array( [[385.2106018066406, 0.0, 318.61053466796875], [0.0, 385.2106018066406, 242.80691528320312], [0.0, 0.0, 1.0]] )
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [[385.2106018066406, 0.0, 318.61053466796875], [0.0, 0.0, 385.2106018066406], [242.80691528320312, 0.0, 0.0, 0.0, 1.0, 0.0]]


print("===")
obj_pts = []
for i in range(ncols-1-1, 0-1, -1): # rows
    for j in range(0, nrows-1, +1): # cols
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
# cv2.solvePnP(model_points, image_points, camera_matrix, D, flags=cv2.CV_ITERATIVE)
corners = np.array(corners)
# ret,rvecs, tvecs = cv2.solvePnP(obj_pts, corners, K, D, flags=cv2.CV_ITERATIVE)
ret,rvecs, tvecs = cv2.solvePnP(obj_pts, corners, K, D, flags=cv2.cv2.SOLVEPNP_ITERATIVE)

print("===")
print(rvecs)
print("---")
rvecs_mat = cv2.Rodrigues(rvecs)
print(rvecs_mat)
print("---")
print(tvecs)


# obj_pts_axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,0.1]]).reshape(-1,3)
obj_pts_axis = np.float32([[0,0,0], [0.0185,0.0185,0], [3*0.0185,3*0.0185,0], [(ncols-1)*0.0185,(nrows-1)*0.0185,0]]).reshape(-1,3)
imgpts, jac = cv2.projectPoints(obj_pts_axis, rvecs, tvecs, K, D)
print(imgpts)
cv2.circle(img, tuple(imgpts[0].ravel()), 5, (255,0,0), 2)
cv2.circle(img, tuple(imgpts[1].ravel()), 5, (0,255,0), 2)
cv2.circle(img, tuple(imgpts[2].ravel()), 5, (0,0,255), 2)
cv2.circle(img, tuple(imgpts[3].ravel()), 5, (255,0,255), 2)
plt.imshow(img)
plt.show()






# https://answers.opencv.org/question/209890/camera-pose-from-checkerboard-pose/
# https://answers.opencv.org/question/209890/camera-pose-from-checkerboard-pose/
# https://answers.opencv.org/question/209890/camera-pose-from-checkerboard-pose/
# https://answers.opencv.org/question/209890/camera-pose-from-checkerboard-pose/

# https://docs.opencv.org/3.4/d7/d53/tutorial_py_pose.html
# https://docs.opencv.org/3.4/d7/d53/tutorial_py_pose.html
# https://docs.opencv.org/3.4/d7/d53/tutorial_py_pose.html
# https://docs.opencv.org/3.4/d7/d53/tutorial_py_pose.html






'''
** NOTES:


*check if saved witout bgr8 what happens


* Directly taking inverse won't work as explained below
* Solving for such a transformation/projection is via cv2.solvePnp...which finds DLT's linear eqns solution
# q = corners[47][0]
# q = np.array(q)
# q = np.transpose(q)
# Q = [[0], [nrows*sq_len]]
# Q = np.array(Q)
# q = s*M*[R|t]*Q
# homog = np.linalg.inv(K)*q*np.linalg.inv(Q) # inverse of a vector Q can't be taken/has no meaning. Need to solve as a system of linear eqns instead!!


* What is the meaining of this axis plot??
* How does it even work? Does it even work??
# def draw(img, corners, imgpts, idx):
#     # corner = tuple(corners[0].ravel())
#     # corner = tuple(corners[10].ravel())
#     # corner = tuple(corners[42].ravel())
#     corner = tuple(corners[idx].ravel())
#     img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
#     img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
#     img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
#     return img

# # Plot axes
# # obj_pts_axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
# obj_pts_axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,0.1]]).reshape(-1,3)
# imgpts, jac = cv2.projectPoints(obj_pts_axis, rvecs, tvecs, K, D)
# print(img.shape)
# print(imgpts)
# img = draw(img, corners, imgpts, 0)
# # img = draw(img, corners, imgpts, 6)
# # img = draw(img, corners, imgpts, 12)
# # img = draw(img, corners, imgpts, 18)
# # img = draw(img, corners, imgpts, 24)
# img = draw(img, corners, imgpts, 27)
# # img = draw(img, corners, imgpts, 30)
# # img = draw(img, corners, imgpts, 36)
# img = draw(img, corners, imgpts, 42)
# plt.imshow(img)
# plt.show()



'''