#!/usr/bin/env python
import os

import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image

from EmoPy.src.fermodel import FERModel

class EmotionRecognizer(object):

    def __init__(self):
        self.sub = rospy.Subscriber('camera/image_raw', Image, self.recognize)
        self.pub = rospy.Publisher('emotion_image', Image, queue_size=1)
        self.count = 0
        self.bridge = CvBridge()
        self.target_emotions = ['calm', 'anger', 'happiness']

    def recognize(self, msg):
        self.count += 1
        if self.count % 5 != 0:
            self.pub.publish(msg)
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            faces = emopy.get_faces(frame) # faces: [(left, top, right, bottom), (...)]
            if not faces:
                return
            biggest_face = max(faces, key=lambda rect: (rect[1][0]-rect[0][0])*(rect[1][1]-rect[0][1]))
            emotions = emopy.recognize(frame, [biggest_face])
            frame = emopy.overlay(frame, [biggest_face], emotions)
            emotion = emotions[0]
            ros_frame = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.pub.publish(ros_frame)
        except CvBridgeError as ex:
            print(ex)

if __name__ == '__main__':
    rospy.init_node('emotion')
    EmotionRecognizer()
    while not rospy.is_shutdown():
        rospy.spin()
