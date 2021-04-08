import pdb

import cv2
import copy
import numpy as np
from matplotlib import pyplot as plt

path = "/home/rxth/catkin_ws/src/CameraProjectorProjection/camera_projector/data/images/"
filename = "img_from_topic.png"

bRodriguesFirstComp = True

img = plt.imread(path+filename, "bgr8") # If format to read it in is not mentioned, then pixels are in 0-1 range.
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# plt.imshow(img_gray, cmap="gray")
# plt.show()

nrows = 7
ncols = 9
pattern_dims = (nrows-1, ncols-1)
ret, corners = cv2.findChessboardCorners(img_gray, pattern_dims, None)
img_corners = copy.deepcopy(img)
if(ret):
    # print(corners)
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
print("OBJ PTS")
obj_pts = []
for i in range(ncols-1-1, 0-1, -1): # rows
    for j in range(0, nrows-1, +1): # cols
        # print([i, j, 0])
        obj_pts.append([i*sq_len, j*sq_len, 0])
obj_pts = np.array(obj_pts)
# print(obj_pts)


## Cross check intrinsic parameters using live data
objpoints = np.array([obj_pts], dtype="float32")
imgpoints = np.array([corners], dtype="float32")
ret_, mtx_, dist_, rvecs_, tvecs_ = cv2.calibrateCamera(objpoints, imgpoints, img_gray.shape[::-1], None, None)
print("===")
print("D")
print(D)
print("D_crosschk")
print(dist_)
print("K")
print(K)
print("K_crosschk")
print(mtx_)
print("R")
print(R)
print("P")
print(P)
print("R and P __crosschk")
print( cv2.Rodrigues(np.array(rvecs_))[0] )
print(tvecs_)


corners = np.array(corners)
ret,rvecs, tvecs = cv2.solvePnP(obj_pts, corners, K, D, flags=cv2.cv2.SOLVEPNP_ITERATIVE)
rvecs_mat = cv2.Rodrigues(rvecs)

print("===")
print("RVECS")
print(rvecs)
print("---")
print("RMAT")
print(rvecs_mat)
print("---")
print("TVECS")
print(tvecs)

# obj_pts_axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,0.1]]).reshape(-1,3)
obj_pts_axis = np.float32([[0,0,0], [0.0185,0.0185,0], [3*0.0185,3*0.0185,0], [(ncols-1)*0.0185,(nrows-1)*0.0185,0]]).reshape(-1,3)
imgpts, jac = cv2.projectPoints(obj_pts_axis, rvecs, tvecs, K, D)
print("===")
print("IMG PTS")
print(imgpts)
cv2.circle(img, tuple(imgpts[0].ravel()), 5, (255,0,0), 2)
cv2.circle(img, tuple(imgpts[1].ravel()), 5, (0,255,0), 2)
cv2.circle(img, tuple(imgpts[2].ravel()), 5, (0,0,255), 2)
cv2.circle(img, tuple(imgpts[3].ravel()), 5, (255,0,255), 2)
plt.imshow(img)
plt.show()


print("========================")
print("PROJECTOR CALIBRATION")
img = plt.imread(path+"squares_scrnshot_1920x1080.png", "bgr8")
img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR) # Convert from four channel BGR-alpha img to three channel BGR img
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# plt.imshow(img_gray, cmap="gray")
# plt.show()

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

# Corner points are detected in the opposite order in the screenshot, 
# so will label object points in a different order
print("===")
print("OBJ PTS")
obj_pts = []
for i in range(0, ncols-1, +1): # rows
    for j in range(nrows-1-1, 0-1, -1): # cols
        # print([i, j, 0])
        obj_pts.append([i*sq_len, j*sq_len, 0])
obj_pts = np.array(obj_pts)
# print(obj_pts)
objpoints = np.array([obj_pts], dtype="float32")
imgpoints = np.array([corners], dtype="float32")
ret, K_pr, D_pr, rvecs_pr, tvecs_pr = cv2.calibrateCamera(objpoints, imgpoints, img_gray.shape[::-1], None, None)
rvecs_pr = rvecs_pr[0]
rvecs_mat_pr = cv2.Rodrigues(rvecs_pr)
tvecs_pr = tvecs_pr[0]

print("===")
print("D_pr")
print(D_pr)
print("K_pr")
print(K_pr)
print("R and P __pr")
print( rvecs_mat_pr )
print(tvecs_)

# If intrinsics already known, can obtain extrinsincs using this:
# ret,rvecs_pr, tvecs_pr = cv2.solvePnP(obj_pts, corners, K_pr, D_pr, flags=cv2.cv2.SOLVEPNP_ITERATIVE)

imgpts, jac = cv2.projectPoints(obj_pts_axis, rvecs_pr, tvecs_pr, K_pr, D_pr)

print("===")
print("IMG PTS")
print(imgpts)
cv2.circle(img, tuple(imgpts[0].ravel()), 15, (255,0,0), 5)
cv2.circle(img, tuple(imgpts[1].ravel()), 15, (0,255,0), 5)
cv2.circle(img, tuple(imgpts[2].ravel()), 15, (0,0,255), 5)
cv2.circle(img, tuple(imgpts[3].ravel()), 15, (255,0,255), 5)
plt.imshow(img)
plt.show()




print("================================================")
print("TRANSFORMATION BETWEEN PROJECTOR AND CAMERA")
if(bRodriguesFirstComp):
    rvecs_mat = rvecs_mat[0]
    rvecs_mat_pr = rvecs_mat_pr[0]

homo_o_p = np.zeros((4, 4))
homo_o_p[0:3, 0:3] = rvecs_mat_pr
homo_o_p[0:3, 3] = tvecs_pr.reshape(3) # https://stackoverflow.com/questions/17869840/numpy-vector-n-1-dimension-n-dimension-conversion
homo_o_p[3][3] = 1

homo_o_c = np.zeros((4,4))
homo_o_c[0:3, 0:3] = rvecs_mat
homo_o_c[0:3, 3] = tvecs.reshape(3)
homo_o_c[3][3] = 1

tmp_trans = homo_o_c[0:3, 3]
tmp_rot = homo_o_c[0:3, 0:3]
tmp_rot_inv = np.linalg.inv(tmp_rot)
homo_c_o = np.zeros((4,4))
homo_c_o[0:3, 0:3] = tmp_rot_inv
homo_o_c[0:3, 3] = -tmp_rot_inv.dot(tmp_trans) # tmp_rot_inv * tmp_trans

homo_c_p = homo_o_p * homo_c_o

print("Camera in Projector's Frame: H_c_p")
print(homo_c_p)

'''
** NOTES:

*Source:
# https://answers.opencv.org/question/209890/camera-pose-from-checkerboard-pose/
# https://docs.opencv.org/3.4/d7/d53/tutorial_py_pose.html


* Check what happens if saved/loaded without "bgr8".


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


# Trying to Improve the contrast of the image...incase the corners can't be detected.
# But that is not needed.
# hist, bins = np.histogram(img_gray.flatten(), 256, [0,256])
# cdf = hist.cumsum()
# cdf_normalized = cdf * hist.max()/ cdf.max()

# plt.plot(cdf_normalized, color = 'b')
# plt.hist(img_gray.flatten(),256,[0,256], color = 'r')
# plt.xlim([0,256])
# plt.legend(('cdf','histogram'), loc = 'upper left')
# plt.show()
# img_gray_histEq = cv2.equalizeHist(img_gray)
# plt.imshow("img", img_gray_histEq)
# plt.show()

'''