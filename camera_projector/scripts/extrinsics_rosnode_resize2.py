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

    def execute(self):
        path = "/home/rxth/catkin_ws/src/CameraProjectorProjection/camera_projector/data/images/"
        filename_aruco = "white_bg_aruco1.png"
        # filename_aruco = "tmp_image.png"

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

        print("Aruco Corners in 3D: ")
        print(object_corners)


        print("Projector Corner Reprojection: ")

       #  K = np.array( [[614.62353515625, 0.0, 327.8028869628906], [0.0, 614.45849609375, 238.79359436035156], [0.0, 0.0, 1.0]] )
        K = np.array( [[ 1358.0929142924545, 0.0, 949.28114114273274], [0.0, 1360.6748913628605, 532.03144707895456], [0.0, 0.0, 1.0 ]] )
        D = np.array( [ 0.17198306778548356, -0.15695033321750945, -0.0065613334239603060, 0.0035590493094028669, 0.0] )

        K_pr = np.array( [[1329.73, 0.0, 420.59], [0.0, 1322.18, 395.68], [0.0, 0.0, 1.0 ]] )
        D_pr = np.array( [0.56581469574959975, -3.8373751872066872, -0.017805365171805934, -0.025716309863278747, 0.0])
        
        R = np.array( [ [0.99957512114540603, -0.023479623897468518, 0.017270913374231957], 
            [0.020922515744415292, 0.99052929754749930, 0.13569804359305362],
            [-0.020293484719768884, -0.13527903740672839, 0.99059969539478332] ] )
        Rinv = np.array([ [0.99957512,  0.02092252, -0.02029348], 
            [-0.02347962,  0.9905293 , -0.13527904], 
            [0.01727091,  0.13569804,  0.9905997] ] )

        t = np.array( [ 0.10171584394938355, -0.18582068182488356, 0.12325087629628882 ] )
        tinv = np.array([-0.0952836, 0.20312234, -0.0986335 ])

        point_tl = R.dot(object_corners[0, :]) + t
        point_tr = R.dot(object_corners[1, :]) + t
        point_br = R.dot(object_corners[2, :]) + t
        point_bl = R.dot(object_corners[3, :]) + t
       #  point_tl = R.dot(object_corners[0, :]) - t
       #  point_tr = R.dot(object_corners[1, :]) - t
       #  point_br = R.dot(object_corners[2, :]) - t
       #  point_bl = R.dot(object_corners[3, :]) - t
        # point_tl = R.dot(object_corners[0, :]) - R.dot(t)
        # point_tr = R.dot(object_corners[1, :]) - R.dot(t)
        # point_br = R.dot(object_corners[2, :]) - R.dot(t)
        # point_bl = R.dot(object_corners[3, :]) - R.dot(t)
        # point_tl = Rinv.dot(object_corners[0, :]) - Rinv.dot(tinv)
        # point_tr = Rinv.dot(object_corners[1, :]) - Rinv.dot(tinv)
        # point_br = Rinv.dot(object_corners[2, :]) - Rinv.dot(tinv)
        # point_bl = Rinv.dot(object_corners[3, :]) - Rinv.dot(tinv)

        points = np.array([point_tl, point_tr, point_br, point_bl], dtype="float32")
        print("Projector's Aruco Corners in 3D: ")
        print(points)

        # reproj, jac = cv2.projectPoints(object_corners, R, t, K_pr, D_pr)
        # reproj, jac = cv2.projectPoints(object_corners, Rinv, tinv, K_pr, D_pr)
        #  reproj, jac = cv2.projectPoints(points, np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), np.array([0.0, 0.0, 0.0]), K_pr, D_pr)
       #  reproj, jac = cv2.projectPoints(object_corners, np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), np.array([0.0, 0.0, 0.0]), K, D)
        reproj, jac = cv2.projectPoints(points, np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), np.array([0.0, 0.0, 0.0]), K, D)

        
        
        # print("Projector 2D Points")
        # print(K_pr.dot(point_tl))

        img_aruco = plt.imread(path+filename_aruco, "bgr8")
        plt.imshow(img_aruco)
        # plt.show()
        img_aruco = cv2.cvtColor(img_aruco, cv2.COLOR_BGRA2BGR) # Convert from four channel BGR-alpha img to three channel BGR img
        cv2.circle(img_aruco, tuple(reproj[0].ravel()), 15, (255,0,0), 5)
        cv2.circle(img_aruco, tuple(reproj[1].ravel()), 15, (0,255,0), 5)
        cv2.circle(img_aruco, tuple(reproj[2].ravel()), 15, (0,0,255), 5)
        cv2.circle(img_aruco, tuple(reproj[3].ravel()), 15, (255,0,255), 5)

        cv2.imwrite(path+"projector_reprojection.png", img_aruco)
        plt.imshow(img_aruco)
        plt.show()

        print("Done Saving. Exiting...")
        print(reproj)

if __name__ == "__main__":
    rospy.init_node('project_corners', anonymous=True)

    pc = ProjectCorners()
    pc.execute()












"""
>>> R = np.array( [ [0.99957512114540603, -0.023479623897468518, 0.017270913374231957], [0.020922515744415292, 0.99052929754749930, 0.13569804359305362], [-0.020293484719768884, -0.13527903740672839, 0.99059969539478332] ] )
>>> 
>>> 
>>> t = np.array( [ 0.10171584394938355, -0.18582068182488356, 0.12325087629628882 ] )
>>> 
>>> 
>>> H = np.zeros(4,4)
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
TypeError: Cannot interpret '4' as a data type
>>> H = np.zeros((4,4))
>>> 
>>> 
>>> H
array([[0., 0., 0., 0.],
       [0., 0., 0., 0.],
       [0., 0., 0., 0.],
       [0., 0., 0., 0.]])
>>> 
>>> 
>>> 
>>> H[0:3, 0:3] = R
>>> 
>>> H[:, 3] = t
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
ValueError: could not broadcast input array from shape (3) into shape (4)
>>> 
>>> 
>>> t
array([ 0.10171584, -0.18582068,  0.12325088])
>>> 
>>> 
>>> t.shape
(3,)
>>> t.reshape(3,1)
array([[ 0.10171584],
       [-0.18582068],
       [ 0.12325088]])
>>> t
array([ 0.10171584, -0.18582068,  0.12325088])
>>> tT = t.reshape(3,1)
>>> 
>>> 
>>> H[:, 3] = tT
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
ValueError: could not broadcast input array from shape (3,1) into shape (4)
>>> 
>>> 
>>> tT
array([[ 0.10171584],
       [-0.18582068],
       [ 0.12325088]])
>>> H[:,3]
array([0., 0., 0., 0.])
>>> H[0:3,3] = tT
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
ValueError: could not broadcast input array from shape (3,1) into shape (3)
>>> 
>>> 
>>> 
>>> H[0:3,3] = t
>>> 
>>> 
>>> 
>>> H
array([[ 0.99957512, -0.02347962,  0.01727091,  0.10171584],
       [ 0.02092252,  0.9905293 ,  0.13569804, -0.18582068],
       [-0.02029348, -0.13527904,  0.9905997 ,  0.12325088],
       [ 0.        ,  0.        ,  0.        ,  0.        ]])
>>> H[3,3] = 1
>>> 
>>> 
>>> H
array([[ 0.99957512, -0.02347962,  0.01727091,  0.10171584],
       [ 0.02092252,  0.9905293 ,  0.13569804, -0.18582068],
       [-0.02029348, -0.13527904,  0.9905997 ,  0.12325088],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
>>> 
>>> 
>>> 
>>> 
>>> np.linalg.inv(H)
array([[ 0.99957512,  0.02092252, -0.02029348, -0.0952836 ],
       [-0.02347962,  0.9905293 , -0.13527904,  0.20312234],
       [ 0.01727091,  0.13569804,  0.9905997 , -0.0986335 ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

"""