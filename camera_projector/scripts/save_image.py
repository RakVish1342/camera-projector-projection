#! /usr/bin/env python
"""
Save an image to file from a ros topic.

Since rospy dosn't have spinOnce() like ros c++, 
for a one-off (vvv rare) event, can use rospy.wait_for_message()
https://answers.ros.org/question/110336/python-spin-once-equivalent/
"""

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class SaveImage:

    def __init__(self):
        self.path = "/home/rxth/catkin_ws/src/CameraProjectorProjection/camera_projector/data/images/"
        self.filename = "img_from_topic.png"
        self.topic = "/camera/color/image_raw"
        # self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.getImage) # rospy has no way of doing spinOnce()

    def execute(self):
        bridge = CvBridge()

        print("Waiting for " + self.topic + " topic...")
        msg = rospy.wait_for_message(self.topic, Image, timeout=None)
        print("Received message from topic.")
        
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(self.path+self.filename, img)
        print("Done Saving. Exiting...")

if __name__ == "__main__":
    rospy.init_node('save_image', anonymous=False)

    si = SaveImage()
    si.execute()
