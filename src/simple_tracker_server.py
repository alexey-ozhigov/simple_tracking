#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import Image, RegionOfInterest
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from cvutils import mono_edgemap
import cv2
from cv2.cv import CV_FOURCC, RGB
import numpy as np
import os
from sys import exit, argv, stderr
from myutils import draw_cross, draw_debug_messages, \
               hsv_filter_mask, calc_back_proj, halt
from mytypes import Enum, State, OneshotTimer
from tracker_lib import TrackerRGBD
from time import time

NODE_NAME = 'simple_tracker_server'

COLOR_TOPIC_DEF     = '/camera/rgb/image_color'
#DEPTH_TOPIC_DEF    = '/camera/depth_registered/image_rect'
DEPTH_TOPIC_DEF     = '/camera/depth/image_rect'
FACE_ROI_TOPIC_DEF  = '/face_roi'

RGB_WND = 'Tracker RGB'
DEPTH_WND = 'Tracker Depth'
DISP_WAIT_TIME_MS = 1
FACE_EDGE_THRESHOLD = 0.05
FACE_DETECTED_TIMEOUT_SEC = 3
FACE_ROI_WIDTH_EXTENTION_K = 0.4
FACE_ROI_HEIGHT_EXTENTION_K = 0.6



class TrackerRGBDServer:
    def __init__(self, tracker):
        rospy.init_node(NODE_NAME)
        self.rgb_subscriber = rospy.Subscriber(COLOR_TOPIC_DEF, Image, self.rgb_cb)
        self.depth_subscriber = rospy.Subscriber(DEPTH_TOPIC_DEF, Image, self.depth_cb)
        self.face_roi_subscriber = rospy.Subscriber(FACE_ROI_TOPIC_DEF, RegionOfInterest, self.face_roi_cb)
        self.face_roi = None
        self.face_timer = OneshotTimer(rospy.Duration(FACE_DETECTED_TIMEOUT_SEC))
        self.tracker = tracker
        self.state = State(['INIT', 'FACE_DETECTED', 'NO_FACE', 'LOST'], 'INIT')
        self.bridge = CvBridge()
        cv2.namedWindow(RGB_WND)
        cv2.namedWindow(DEPTH_WND)
        self.show_empty_images()
        rospy.loginfo('TrackerRGBDServer initialized')

    def display_image(self, wnd, img):
        cv2.imshow(wnd, img)
        cv2.waitKey(DISP_WAIT_TIME_MS)
        
    def show_empty_images(self):
        img = 255 * np.ones((640, 480), dtype='uint8')
        draw_debug_messages(img, ['NO IMAGE RECEIVED'])
        self.display_image(RGB_WND, img)
        self.display_image(DEPTH_WND, img)

    def cv_image_from_ros_msg(self, msg, enc='bgr8'):
        try:
            img = self.bridge.imgmsg_to_cv(msg, enc)
            img = np.asarray(img)
        except CvBridgeError, e:
            print e
        return img

    def rgb_cb(self, imgm):
        img = self.cv_image_from_ros_msg(imgm)
        #self.display_image(RGB_WND, img)

    def extended_face_roi(self, face_roi, img_shape):
        extroi = RegionOfInterest()
        wext = int(face_roi.width * FACE_ROI_WIDTH_EXTENTION_K)
        hext = int(face_roi.height * FACE_ROI_HEIGHT_EXTENTION_K)
        extroi.x_offset = max(face_roi.x_offset - wext, 0)
        extroi.y_offset = max(face_roi.y_offset - hext, 0)
        extroi.width = min(face_roi.width + 2*wext, img_shape[1] - extroi.x_offset - 1)
        extroi.height = min(face_roi.height + 2*hext, img_shape[0] - extroi.y_offset - 1)
        return extroi

    def depth_cb(self, imgm):
        img = self.cv_image_from_ros_msg(imgm, '32FC1')
        if self.state == 'INIT':
            pass
        elif self.state == 'FACE_DETECTED':
            if self.no_face_recently():
                self.state.set('NO_FACE')
            else:
                extroi = self.extended_face_roi(self.face_roi, img.shape)
                roi_img = img[extroi.y_offset: extroi.y_offset + extroi.height,
                              extroi.x_offset: extroi.x_offset + extroi.width]
                edgemap = mono_edgemap(roi_img, thr=FACE_EDGE_THRESHOLD)
                self.display_image(DEPTH_WND, edgemap)

    def no_face_recently(self):
        assert self.state != 'INIT' 
        return self.face_timer.timedout

    def face_roi_cb(self, roim):
        self.face_roi = roim
        #rospy.loginfo('.')
        if self.state == 'INIT':
            self.state.set('FACE_DETECTED')
            self.face_timer.set()
        elif self.state == 'FACE_DETECTED':
            self.face_timer.reset()
        elif self.state == 'NO_FACE':
            self.state.set('FACE_DETECTED')
            self.face_timer.reset()
        #rospy.loginfo('face %d %d %d %d' % (roim.x_offset, roim.y_offset, roim.width, roim.height))

    def run(self):
        rospy.loginfo('TrackerRGBDServer running')
        rospy.spin()

    @staticmethod
    def on_shutdown():
        cv2.destroyAllWindows()
        cv2.waitKey(30)
        rospy.loginfo('TrackerRGBDServer shutdown')


if __name__ == '__main__':
    tracker = TrackerRGBD()
    tracker_serv = TrackerRGBDServer(tracker)
    rospy.on_shutdown(TrackerRGBDServer.on_shutdown)
    tracker_serv.run()

