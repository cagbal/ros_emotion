#!/usr/bin/env python

import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2
import dlib
import numpy as np

from EmoPy.src.fermodel import FERModel

class EmotionRecognizer(object):

    def __init__(self, emotions=['disgust', 'happiness', 'surprise', ]):
        self.sub = rospy.Subscriber('/body_tracker/rgb/raw_image', Image, self.recognize)
        self.pub = rospy.Publisher('emotion_image', numpy_msg(Image), queue_size=0)
        self.count = 0
        self.target_emotions = emotions
        self.detector = dlib.get_frontal_face_detector()
        self.model = FERModel(self.target_emotions, verbose=True)

    def recognize(self, msg):
        try:
            frame = np.frombuffer(
                msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

            dets = self.detector(frame, 1)

            assert len(dets) > 0, "There is no face!"

            for d in dets:
                self.model.predict(
                    frame[d.top():d.bottom(), d.left():d.right()])
                cv2.imshow("a", frame[d.top():d.bottom(), d.left():d.right()])
                cv2.waitKey(1)
            #self.pub.publish(ros_frame)
                break
        except Exception as e:
            print("Error: " + str(e))

if __name__ == '__main__':
    rospy.init_node('emotion')
    EmotionRecognizer()
    while not rospy.is_shutdown():
        rospy.spin()
