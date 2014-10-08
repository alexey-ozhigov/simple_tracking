#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2.cv import CV_FOURCC, RGB
import numpy as np
import os
from sys import exit, argv, stderr
from myutils import draw_cross, draw_debug_messages, \
               hsv_filter_mask, calc_back_proj, halt
from mytypes import Enum
from camshift_rgbd_lib import CamshiftRGBD

'''
def talker():
    pub = rospy.Publisher('chatter', String)
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
'''

CONDITION_OCC1 = 1
CONDITION_OCC2 = 2
CONDITION_OCC3 = 3

NODE_NAME = 'camshift_rgbd'

COLOR_TOPIC_DEF        = '/camera/rgb/image_color'
#DEPTH_TOPIC_DEF        = '/camera/depth_registered/image_rect'
DEPTH_TOPIC_DEF        = '/camera/depth/image_rect'

RGB_WND = 'Camshift RGB'
DEPTH_WND = 'Camshift Depth'
DISP_WAIT_TIME_MS = 1

class OpticalMarkerTracker:
    """Tracks a single colored blob with [externally] given initial location and size.
       
       Uses two modes of channels utilization: RGBONLY (only RGB image) RGBD (RGB image plus floating point DEPTH map).
       Two phases of operation: INIT (Hue histogram is calculated for the specified window), TRACK (the window is tracked using CAMSHIFT).
       If set_*_topic methods are invoked, the given ROS topics are used as channels and @run method must be used to execute the tracker.
       If set_*_topic methods are not used, @run_video method must be used with specified video file names.
    """

    def __init__(self, channels, markers, tracker):
        self.rgb_frame_cnt = 0
        self.depth_frame_cnt = 0
        self.channels = channels
        self.markers = markers
        self.object = select_object_marker(markers)
        self.tracker = tracker
        self.state = TrackerStates.INIT
        self.rgb_subscriber = None
        self.depth_subscriber = None
        self.debug_pub = None
        self.output_pubs_pos = None
        self.output_pubs_hue = None
        self.subs_pos_occ = None
        self.subs_hue_occ = None
        #results of tracking
        self.vwriter = None
    def cv_image_from_ros_msg(self, msg):
        try:
            img = bridge.imgmsg_to_cv(msg, "bgr8")
            img = np.asarray(img)
        except CvBridgeError, e:
            print e
        return img

    def ros_msg_from_cv_image(self, img):
        try:
            msg = bridge.cv_to_imgmsg(img, "bgr8")
        except CvBridgeError, e:
            print e
        return msg

    def find_markers(self, img):
        obj = select_object_marker(self.markers)
        obj_c = obj.center
        obj_s = obj.size
        self.draw_state_info(img)
        cv2.rectangle(img, (obj_c.x - obj_s.x / 2, obj_c.x + obj_s.x / 2), \
                           (obj_c.y - obj_s.y / 2, obj_c.y + obj_s.y / 2), \
                           RGB(255, 0, 0), 1)
        self.vwriter.write(img)
        #size of ROI 2*width x 2*height
        img_roi = img[obj.center.y - obj.size.y : obj.center.y + obj.size.y, obj.center.x - obj.size.x : obj.center.x + obj.size.x]
        img_hsv = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)
        mask = hsv_filter_mask(img_hsv)
        markers_cands = []
        for m in self.markers:
            if m == obj:
                continue
            c = m.center
            s = m.size
            w = (c.x - s.x / 2, c.y - s.y / 2, s.x, s.y)
            back_proj = calc_back_proj(img_hsv, m.hist.hist, hsv = True)
            back_proj &= mask
            cv2.imwrite('out_blobs/%s_blobs_%03d.png' % (m.id, self.depth_frame_cnt), back_proj)

    def rgb_cb(self, img_data):
        #print 'Processed rgb frame %d' % self.rgb_frame_cnt
        self.rgb_frame_cnt += 1
        if self.rgb_subscriber:#from ROS topic
            img = self.cv_image_from_ros_msg(img_data)
        else:#from video file
            img = img_data
        #print 'Object', self.object.center, self.object.size
        if self.state == TrackerStates.INIT:
            self.initialize(img)
            self.state = TrackerStates.TRACK
            return
        elif self.state == TrackerStates.TRACK:
            #Warning: img will be drawn on by track(.)
            ret_img, markers_hue = self.track(img)
            #publish hue value of each marker's center
            for i in range(len(self.markers)):
                if self.output_pubs_hue:
                    self.output_pubs_hue[i].publish(markers_hue[i])
            if self.debug_pub:
                debug_msg = self.ros_msg_from_cv_image(ret_img)
                self.debug_pub.publish(debug_msg)
        elif self.state == TrackerStates.OCCLUDED:
            markers_cands = self.find_markers(img)
        else:
            raise Exception('Unknown/Unhandled TrackerState: %s' \
                            % self.state)

    def depth_image_cb(self, data):
        #print 'Processed depth frame %d' % self.depth_frame_cnt
        self.depth_frame_cnt += 1
        if self.depth_subscriber:#from ROS topic
            img = self.cv_image_from_ros_msg(data)
        else:#from video file
            img = data
        img_mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        for i in range(len(self.markers)):
            m = self.markers[i]
            m.center.z = img_mono[m.center.y][m.center.x]
            draw_cross(img_mono, (m.center.x + 1, m.center.y + 1), 20, RGB(255, 255, 255), thickness=1)
            draw_debug_messages(img_mono, ['Depth: %dcm' % m.center.z])
            cv2.imshow('depth_wnd', img_mono)
            img_bgr = cv2.cvtColor(img_mono, cv2.COLOR_GRAY2BGR)
            self.vwriter_depth.write(img_bgr)
            if self.output_pubs_pos:
                self.output_pubs_pos[i].publish(m.center)

    def depth_change_cb(self, msg):
        print 'DEPTH CHANGE: %d %s' % (self.depth_frame_cnt, msg.marker_id)
        if self.state == TrackerStates.TRACK:
            self.state = TrackerStates.OCCLUDED

    def hue_change_cb(self, msg):
        print 'HUE CHANGE: %d %s' % (self.depth_frame_cnt, msg.marker_id)
        if self.state == TrackerStates.TRACK:
            pass
            #self.state = TrackerStates.OCCLUDED

    def set_rgb_topic(self, topic_name):
        self.rgb_subscriber = rospy.Subscriber(topic_name, Image, self.rgb_cb)

    def set_depth_image_topic(self, topic_name):
        self.channels = TrackerChannels.RGBD
        self.depth_subscriber = rospy.Subscriber(topic_name, Image, self.depth_image_cb)
 
    def set_debug_topic(self, topic_name):
        self.debug_publisher = rospy.Publisher(topic_name, Image)

    def set_output_topic(self, topic_name_base):
        assert len(self.markers) > 0, 'OpticalMarkerTracker: before envoking @set_output_topic, add markers'
        self.output_pubs_pos = []
        self.output_pubs_hue = []
        self.subs_pos_occ = []
        self.subs_hue_occ = []
        for m in self.markers:
            #pos
            topic_name = topic_name_base + '/' + m.id
            pub = rospy.Publisher(topic_name, Point)
            self.output_pubs_pos.append(pub)
            #hue
            topic_name = topic_name_base + '/' + m.id + '_hue'
            pub = rospy.Publisher(topic_name, Int64)
            self.output_pubs_hue.append(pub)
            #pos occlusion
            topic_name = topic_name_base + '/' + m.id + '_occ'
            print topic_name
            #sub = rospy.Subscriber(topic_name, PositionChange, self.depth_change_cb)
            #self.subs_pos_occ.append(sub)
            #hue occlusion
            topic_name = topic_name_base + '/' + m.id + '_hue_occ'
            print topic_name
            sub = rospy.Subscriber(topic_name, HueChange, self.hue_change_cb)
            self.subs_hue_occ.append(sub)

    def set_debug_video(self, fname):
        self.vwriter = cv2.VideoWriter(fname, \
                               CV_FOURCC('P', 'I', 'M', '1'), \
                               30.0, (640, 480), True)

    def set_debug_prob_video(self, fname):
        self.vwriter_prob = cv2.VideoWriter(fname, \
                               CV_FOURCC('P', 'I', 'M', '1'), \
                               30.0, (640, 480), True)
        
    def set_debug_depth_video(self, fname):
        self.vwriter_depth = cv2.VideoWriter(fname, \
                               CV_FOURCC('P', 'I', 'M', '1'), \
                               30.0, (640, 480), True)

    '''
    def publish_output(self):
        for i in range(len(self.markers)):
            m = self.markers[i]
            topic = self.output_pubs_pos[i]
            msg = Point(x = m.center.x, y = m.center.y, z = m.center.z)
            topic.publish(msg)
    '''

    def initialize(self, img):
        self.tracker.init(img, self.markers)
        #insert_pause_to_video(self.vwriter, img, 60)
 
    def draw_state_info(self, img):
        draw_debug_messages(img, ['State: ' + self.state], orig=(30, 90))

    def track(self, img):
        (ret_img, prob_imgs, markers_hue) = self.tracker.run(img, self.markers)
        self.draw_state_info(img)
        self.vwriter.write(img)
        self.vwriter_prob.write(prob_imgs[0])
        cv2.imshow('rgb_wnd', ret_img)
        cv2.imshow('prob_wnd', prob_imgs[0])
        cv2.waitKey(3)
        return (ret_img, markers_hue)

    def run(self):
        assert len(self.markers) > 0, 'No markers set'
        assert self.rgb_subscriber or self.depth_subscriber, \
               'OpticalMarkerTracker.run() can be used only after set_*_topic'
        rospy.spin()
    
    def run_video(self, rgb_video_file, depth_video_file):
        assert len(self.markers) > 0, 'No markers set'
        assert not self.rgb_subscriber and not self.depth_subscriber, \
               'OpticalMarkerTracker.run_video() cannot be used if set_*_topic was called'
        rgb_capt = cv2.VideoCapture(rgb_video_file)
        init_img = rgb_capt.read()
        if not init_img[0]:
            print >>stderr, 'Cannot read video from "' + rgb_video_file + '"'
            return
        depth_capt = cv2.VideoCapture(depth_video_file)
        frame_i = 1
        paused = False
        while True:
            ch = cv2.waitKey(33)
            if ch == 27:
                rospy.signal_shutdown('Bye!')
                break
            if ch == -1 and paused:
                continue
            if ch == ord(' '):
                if not paused:
                    paused = True
                    continue
                else:
                    paused = False
            rgb_img = rgb_capt.read()
            if not rgb_img[0]:
                break
            depth_img = depth_capt.read()
            if not depth_img[0]:
                break
            self.rgb_cb(rgb_img[1])
            self.depth_image_cb(depth_img[1])
            #self.publish_output()
            frame_i += 1


def depth_image_callback(data):
    try:
        img = bridge.imgmsg_to_cv(data)
        img = np.asarray(img)
        #cv2.imshow('rgb_wnd', img) 
    except CvBridgeError, e:
        print e

def camera_info_callback(data):
    pass
    #rospy.loginfo(rospy.get_name() + 'I heard %s', data.header)

def color_image_callback(data):
    try:
        img = bridge.imgmsg_to_cv(data, "bgr8")
        img = np.asarray(img)
        cv2.imshow('rgb_wnd', img) 
    except CvBridgeError, e:
        print e
    cv2.waitKey(3)

def run_with_input_ros_topics(mtracker, color_topic = COLOR_TOPIC_DEF, depth_topic = DEPTH_TOPIC_DEF):
    mtracker.set_rgb_topic(color_topic)
    mtracker.set_depth_image_topic(depth_topic)
    mtracker.set_debug_topic(DEBUG_TOPIC_DEF)
    mtracker.run()

def run_with_video_files(mtracker, rgb_video_file, depth_video_file):
    mtracker.run_video(rgb_video_file, depth_video_file)

def read_initial_markers(pos_file_name):
    #markers = []
    #markers.append(Marker(center=Point(155, 354, 0), size=Point( 9, 7, 0)))
    #markers.append(Marker(center=Point(116, 398, 0), size=Point(8, 8, 0)))
    markers = []
    marker_lines = file(pos_file_name).readlines()
    marker_pos_re = '%s: center=(%d %d %d) size=(%d %d %d)'
    for l in marker_lines:
        vals = sscanf(l, marker_pos_re)
        if not vals:
            raise Exception('File %s: Invalid marker position: %s' % (pos_file_name, l))
        _id = vals[0]
        center = Point(vals[1], vals[2], vals[3])
        size   = Point(vals[4], vals[5], vals[6])
        markers.append(Marker(id = _id, center=center, size=size))
    return markers

def test():
    markers = read_initial_markers('/home/alex/MAS/RnD1/kinect/simple_packages/simple_pcl/bags/hand_rgb.init_pos')
    for m in markers:
        print m

def run_optical_markers_detector():
    assert len(argv) >= 5, \
      'USAGE: simple_2ddepth <rgb_channel> <depth_channel> ros_topic|video_file'

    rospy.init_node('simple_2ddepth', anonymous=True)
    print 'Opening %s' % argv[4]
    markers = read_initial_markers(argv[4])
    camshift = CamShift()
    mtracker = OpticalMarkerTracker(channels=TrackerChannels.RGBONLY, markers=markers, tracker=camshift)
    mtracker.set_output_topic(OUTPUT_TOPIC_DEF)

    rgb_channel = argv[1]
    depth_channel = argv[2]
    if argv[3] == 'false':#inputs are video files
        debug_vfname = dirname(rgb_channel) + '/' + basename(rgb_channel).split('_')[0]
        mtracker.set_debug_video(debug_vfname + '_' + VIDEO_DEBUG_FILE)
        mtracker.set_debug_prob_video(debug_vfname + '_' + VIDEO_DEBUG_PROB_FILE)
        mtracker.set_debug_depth_video(debug_vfname + '_' + VIDEO_DEBUG_DEPTH_FILE)
        run_with_video_files(mtracker, rgb_channel, depth_channel)
        sleep(2)
        os.system('./term_listeners.sh')
        sleep(2)
        rospy.signal_shutdown('Finished!')
    elif argv[3] == 'true':#inputs are ROS topics
        mtracker.set_debug_video(VIDEO_DEBUG_FILE)
        mtracker.set_debug_prob_video(VIDEO_DEBUG_PROB_FILE)
        run_with_input_ros_topics(mtracker, rgb_channel, depth_channel)
    else:
        assert False, 'Unknown ros_input: "%s", must be "true" or "false"' % argv[3]

class CamshiftRGBDServer:
    def __init__(self, camshift):
        rospy.init_node(NODE_NAME)
        self.rgb_subscriber = rospy.Subscriber(COLOR_TOPIC_DEF, Image, self.rgb_cb)
        self.depth_subscriber = rospy.Subscriber(DEPTH_TOPIC_DEF, Image, self.depth_cb)
        self.camshift = camshift
        self.bridge = CvBridge()
        cv2.namedWindow(RGB_WND)
        cv2.namedWindow(DEPTH_WND)
        self.show_empty_images()
        rospy.loginfo('CamshiftRGBDServer initialized')

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
        rospy.loginfo('rgb .')
        img = self.cv_image_from_ros_msg(imgm)
        self.display_image(RGB_WND, img)

    def depth_cb(self, imgm):
        img = self.cv_image_from_ros_msg(imgm, '32FC1')
        self.display_image(DEPTH_WND, img)

    def run(self):
        rospy.loginfo('CamshiftRGBDServer running')
        rospy.spin()

    @staticmethod
    def on_shutdown():
        cv2.destroyAllWindows()
        cv2.waitKey(30)
        rospy.loginfo('CamshiftRGBDServer shutdown')


if __name__ == '__main__':
    camshift = CamshiftRGBD()
    camshift_serv = CamshiftRGBDServer(camshift)
    rospy.on_shutdown(CamshiftRGBDServer.on_shutdown)
    camshift_serv.run()

