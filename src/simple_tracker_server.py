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
import pickle

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
FACE_DEPTH_DELTA_M = 0.05
DEPTH_FOR_NAN_M = 20


class Blob:
    def __init__(self, center, hue):
        self.center = center
        self.hue = hue
    
    @staticmethod
    def from_rgbd(rgb_img, depth_img, delta):
        #1. Extract pixels approx. at the face distance
        H, W = depth_img.shape[0:2]
        ref_depth = depth_img[H/2, W/2]
        #TODO: make sure ref_depth is valid (not nan)
        m = np.max(depth_img)
        depth_img[np.isnan(depth_img)] = DEPTH_FOR_NAN_M
        points_further = np.where(depth_img > ref_depth - delta)
        points_closer = np.where(depth_img < ref_depth + delta)
        points_merged_further = zip(points_further[0], points_further[1])
        points_merged_closer = zip(points_closer[0], points_closer[1])
        points_merged = zip(*list(set.intersection(set(points_merged_further), set(points_merged_closer))))
        #2. Go from the center upwards collecting hue values along the line
        px, py = W/2, H/2
        #all rgb pixels if their depth is not too big (e.g. distant objects up and beyond the head)
        vert_rgb = np.array([[rgb_img[y, px] for y in range(0, py+1) if depth_img[y, px] < ref_depth + delta]])
        vert_hue = cv2.split(cv2.cvtColor(vert_rgb, cv2.COLOR_BGR2HSV))[0]
        hist = np.histogram(vert_hue, bins=30)
        rospy.signal_shutdown(0)

        dbg = np.zeros_like(rgb_img)
        dbg[points_closer] = rgb_img[points_closer]
        cv2.namedWindow('dbg')
        cv2.imshow('dbg', dbg)
        cv2.waitKey(DISP_WAIT_TIME_MS)

class TrackerRGBDServer:
    def __init__(self, tracker):
        rospy.init_node(NODE_NAME)
        self.rgb_subscriber = rospy.Subscriber(COLOR_TOPIC_DEF, Image, self.rgb_cb)
        self.depth_subscriber = rospy.Subscriber(DEPTH_TOPIC_DEF, Image, self.depth_cb)
        self.face_roi_subscriber = rospy.Subscriber(FACE_ROI_TOPIC_DEF, RegionOfInterest, self.face_roi_cb)
        self.face_roi = None
        self.first_face_rgb = None   #for blob initialization
        self.first_face_depth = None #for blob initialization
        self.first_face_roi = None   #for blob initialization
        self.face_timer = OneshotTimer(rospy.Duration(FACE_DETECTED_TIMEOUT_SEC))
        self.tracker = tracker
        self.state = State(['INIT', 'FIRST_FACE_DETECTED', 'FACE_DETECTED', 'NO_FACE', 'LOST'], 'INIT')
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
        if self.state == 'FIRST_FACE_DETECTED':
            if self.first_face_roi is None:
                return
            extroi = self.extended_face_roi(self.first_face_roi, img.shape)
            roi_img = img[extroi.y_offset: extroi.y_offset + extroi.height,
                          extroi.x_offset: extroi.x_offset + extroi.width]
            self.first_face_rgb = roi_img.copy()
            if self.first_face_depth is not None:
                self.blobs = Blob.from_rgbd(self.first_face_rgb, self.first_face_depth, FACE_DEPTH_DELTA_M)
                self.state.set('FACE_DETECTED')
            else:
                return
        self.rgb_img = img.copy()
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
        elif self.state == 'FIRST_FACE_DETECTED':
                #extract face depthmap, then if corresp. rgb available - find blobs
                if self.first_face_roi is None:
                    return
                extroi = self.extended_face_roi(self.first_face_roi, img.shape)
                roi_img = img[extroi.y_offset: extroi.y_offset + extroi.height,
                              extroi.x_offset: extroi.x_offset + extroi.width]
                self.first_face_depth = roi_img
                if self.first_face_rgb is not None:
                    self.blobs = Blob.from_rgbd(self.first_face_rgb, self.first_face_depth, FACE_DEPTH_DELTA_M)
                    self.state.set('FACE_DETECTED')
                else:
                    return
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
        #rospy.loginfo('.')
        self.face_roi = roim
        if self.state == 'INIT':
            self.state.set('FIRST_FACE_DETECTED')
        elif self.state == 'FIRST_FACE_DETECTED':
            if self.first_face_roi is None:
                self.first_face_roi = roim
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

