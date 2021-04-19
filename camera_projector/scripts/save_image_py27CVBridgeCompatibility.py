#! /usr/bin/env python
"""
Save an image to file from a ros topic.

Since rospy dosn't have spinOnce() like ros c++, 
for a one-off (vvv rare) event, can use rospy.wait_for_message()
https://answers.ros.org/question/110336/python-spin-once-equivalent/

There are issues with cv_bridge and python3 which requires a rebuild
of cv_bridge. extrinsics_node.py requires the use of python3 for some
calibration matrix I think. So, using this node to perform "live/new" 
camera image saving which will then be read by extrinsics_node.py.
"""

import cv2
import rospy
import std_msgs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class SaveImageCompatibility:

    def __init__(self):
        self.path = "/home/rxth/catkin_ws/src/CameraProjectorProjection/camera_projector/data/images/"
        self.filename = "img_from_topic_py27CVBridgeCompatibility.png"
        self.topic = "/camera/color/image_raw"
        # self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.getImage) # rospy has no way of doing spinOnce()
        self.pub = rospy.Publisher("/camera/color/image_raw_py27CVBridgeCompatibility", std_msgs.msg.Bool, queue_size=10)
        self.rate = rospy.Rate(1)

    def execute(self):
        bridge = CvBridge()

        print("Waiting for " + self.topic + " topic...")
        msg = rospy.wait_for_message(self.topic, Image, timeout=None)
        print("Received message from topic.")
        
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(self.path+self.filename, img)
        print("Done Saving. Exiting...")

        # while(not rospy.is_shutdown()):
        #     self.pub.publish(std_msgs.msg.Bool(True))
        #     print("Published py27CVBridgeCompatibility message about saving image.")
        #     self.rate.sleep()



if __name__ == "__main__":
    rospy.init_node('save_image', anonymous=False)

    sic = SaveImageCompatibility()
    sic.execute()
