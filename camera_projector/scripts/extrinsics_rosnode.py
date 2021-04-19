#! /usr/bin/env python
"""

"""
import pdb
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import copy
import math
import numpy as np
import cv2
from matplotlib import pyplot as plt


class ProjectCorners:

    curr_pose = PoseStamped()
    des_pose = PoseStamped()

    wayptIdx = 0
    distThreshold = 0.4
    subDistThresh = 0.4
    subDistStep = 0.3
    # distThreshold = 0.8
    # subDistThresh = 0.8    
    # subDistStep = 0.6
    isReadyToFly = False
    sim_loop_ctr = 1


    def execute(self):
        path = "/home/rxth/catkin_ws/src/CameraProjectorProjection/camera_projector/data/images/"
        filename = "img_from_topic_py27CVBridgeCompatibility.png"
        filename_aruco = "white_bg_aruco1.png"

        bRodriguesFirstComp = True
        bridge = CvBridge()

        # img = plt.imread(path+filename, "bgr8") # If format to read it in is not mentioned, then pixels are in 0-1 range.
        # Trying to calibrate on live image rather than on the stored image which was used long back
        # print("Waiting for topic /camera/color/image_raw_py27CVBridgeCompatibility ...")
        # flag = False
        # while(not flag):
        #     msg = rospy.wait_for_message("/camera/color/image_raw_py27CVBridgeCompatibility", Bool, timeout=None)
        # print("Received image data.")
        # img = bridge.imgmsg_to_cv2(msg, "bgr8")
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
        # plt.show()

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
        print(obj_pts)


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
        # plt.show()






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
        # plt.show()

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
        # plt.show()




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

        

        print("##################################################################################")
        print("##################################################################################")
        print("##################################################################################")
        print("Waiting for topics1...")
        object_corner_tl = rospy.wait_for_message("/camera_aruco/corner_tl", PoseStamped, timeout=None)
        print("Waiting for topics2...")
        object_corner_tr = rospy.wait_for_message("/camera_aruco/corner_tr", PoseStamped, timeout=None)
        print("Waiting for topics3...")
        object_corner_br = rospy.wait_for_message("/camera_aruco/corner_br", PoseStamped, timeout=None)
        print("Waiting for topics4...")
        object_corner_bl = rospy.wait_for_message("/camera_aruco/corner_bl", PoseStamped, timeout=None)
        print("Done waiting.")

        object_corners = np.float32([
            [object_corner_tl.pose.position.x, object_corner_tl.pose.position.y, object_corner_tl.pose.position.z], 
            [object_corner_tr.pose.position.x, object_corner_tr.pose.position.y, object_corner_tr.pose.position.z], 
            [object_corner_br.pose.position.x, object_corner_br.pose.position.y, object_corner_br.pose.position.z], 
            [object_corner_bl.pose.position.x, object_corner_bl.pose.position.y, object_corner_bl.pose.position.z]]).reshape(-1,3)

        print("---OBJ")
        print(object_corners)

        img_corners, jac_corners = cv2.projectPoints(object_corners, rvecs_pr, tvecs_pr, K_pr, D_pr)
        print("===")

        img_aruco = plt.imread(path+filename_aruco, "bgr8")
        plt.imshow(img_aruco)
        plt.show()
        img_aruco = cv2.cvtColor(img_aruco, cv2.COLOR_BGRA2BGR) # Convert from four channel BGR-alpha img to three channel BGR img

        print("IMG PTS")
        print(imgpts)

        # off_x = 40+638
        # off_y = 250+255
        off_x = 620
        off_y = 520
        img_corners[0][0][0] = img_corners[0][0][0] + off_x
        img_corners[0][0][1] = img_corners[0][0][1] + off_y

        img_corners[1][0][0] = img_corners[1][0][0] + off_x
        img_corners[1][0][1] = img_corners[1][0][1] + off_y

        img_corners[2][0][0] = img_corners[2][0][0] + off_x
        img_corners[2][0][1] = img_corners[2][0][1] + off_y

        img_corners[3][0][0] = img_corners[3][0][0] + off_x
        img_corners[3][0][1] = img_corners[3][0][1] + off_y


        cv2.circle(img_aruco, tuple(img_corners[0].ravel()), 15, (255,0,0), 5)
        cv2.circle(img_aruco, tuple(img_corners[1].ravel()), 15, (0,255,0), 5)
        cv2.circle(img_aruco, tuple(img_corners[2].ravel()), 15, (0,0,255), 5)
        cv2.circle(img_aruco, tuple(img_corners[3].ravel()), 15, (255,0,255), 5)
        
        print("---IMG")
        print(img_aruco.shape)
        print(img_corners)
        cv2.imwrite(path+"out.png", img_aruco)
        plt.imshow(img_aruco)
        plt.show()

        print("Done Saving. Exiting...")


if __name__ == "__main__":
    rospy.init_node('project_corners', anonymous=True)

    pc = ProjectCorners()
    pc.execute()
