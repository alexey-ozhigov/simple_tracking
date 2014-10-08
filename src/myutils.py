#Copyright 2012 Alexey Ozhigov.
'''
This file is part of RnD1_Alexey_Ozhigov.
 RnD1_Alexey_Ozhigov is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 RnD1_Alexey_Ozhigov is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 You should have received a copy of the GNU General Public License along with RnD1_Alexey_Ozhigov. If not, see <http://www.gnu.org/licenses/>.
'''




import cv2
from cv import Scalar
import matplotlib as mpl
import matplotlib.pyplot as plt
import re
from sys import exit

from cv2.cv import CV_FOURCC
import numpy as np
from myutils_io import read_image, read_image_from_video

SCALAR_BLACK = Scalar(0, 0, 0)
SCALAR_WHITE = Scalar(255, 255, 255)
SCALAR_GREEN = Scalar(0, 255, 0)
SCALAR_RED   = Scalar(0, 0, 255)

def hue_to_hist(hue_min, hue_max, hist_bins):
    hist = np.array([], dtype='uint8')
    for i in range(hist_bins):
        bin_w = 180.0 / hist_bins
        c = bin_w * (i + 0.5)
        if hue_min < c < hue_max:
            a = c - hue_min
            b = hue_max - c
            min_s = min(a, b)
            max_s = max(a, b)
            val = 255.0 * min_s / max_s
        else:
            val = 0
        hist = np.append(hist, int(val))
    return (255.0 / max(hist) * hist).astype('uint8')


if __name__ == '__main__':
    VIDEO_FILE         = '../hand_3markers/out.avi'
    RED_HIST_FILE      ='../hand_3markers/red_marker_frame51.png.hhst'
    RECT_RED_HIST_FILE ='../hand_3markers/rectangle_red_marker_frame51.png.hhst'
    BLUE_HIST_FILE     ='../hand_3markers/blue_marker_frame51.png.hhst'
    BLUE2_HIST_FILE    ='../pure_markers/hand_3fingers_blue_frame300.png.hhst'
    YELLOW_HIST_FILE   ='../hand_3markers/yellow_marker_frame51.png.hhst'
    HIST_FILES         = [RED_HIST_FILE, BLUE_HIST_FILE, YELLOW_HIST_FILE, \
                      RECT_RED_HIST_FILE]

def draw_debug_messages(img, msgs, orig = (30, 30), color=SCALAR_GREEN, font_scale= 1.0, thickn = 2):
    LINES_INTERVAL = 30
    x, y = orig
    for m in msgs:
        cv2.putText(img, m, (x, y), cv2.FONT_HERSHEY_PLAIN, \
                fontScale = font_scale, thickness = thickn, color = color)
        y += LINES_INTERVAL
def draw_label(img, label, orig = (50, 50), color = SCALAR_BLACK):
    cv2.putText(img, label, orig, cv2.FONT_HERSHEY_PLAIN, \
                    2.0, SCALAR_BLACK)
def draw_bound_rectangle(img, points, border, color):
    rect = cv2.boundingRect(points)
    if rect[2] < rect[3]:
        ratio = 1.0 * rect[2] / rect[3]
    else:
        ratio = 1.0 * rect[3] / rect[2]
    if ratio < 0.8:
        thickness = 1
    else:
        thickness = 2
    cv2.rectangle(img, (rect[0] - border, rect[1] - border), \
                       (rect[0] + rect[2] + border, \
                        rect[1] + rect[3] + border), color, thickness)
    return rect
    
def draw_cross(img, pt, length, color, thickness = 3):
    l2 = length / 2
    cv2.line(img, (pt[0] - l2, pt[1]), (pt[0] + l2, pt[1]), color, thickness)
    cv2.line(img, (pt[0], pt[1] - l2), (pt[0], pt[1] + l2), color, thickness)

def HSV(img_in, x, y):
    if type(img_in) == type(''):
        frame = read_image(fname)
    else:
        frame = img_in
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return {'BGR': frame[x, y], 'HSV': frameHSV[x, y]}
    #return frameHSV[x, y]

#ignore_mask if defined must contain a 3-element np.array with RGB color
# which must not be accounted for when calculated min. and max. values
def MinMaxHSV(img_in, x1, y1, x2, y2, ignore_mask = None):
    if type(img_in) == type(''):
        frame = read_image(fname)
    else:
        frame = img_in
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    MinHSV = [255, 255, 255]
    MaxHSV = [0, 0, 0]
    for i in range(x1, x2):
        for j in range(y1, y2):
            for k in [0, 1, 2]:
                if ignore_mask != None and (frame[j, i] == ignore_mask).all():
                    continue
                if frameHSV[j, i][k] < MinHSV[k]:
                    MinHSV[k] = frameHSV[j, i][k]
                if frameHSV[j, i][k] > MaxHSV[k]:
                    MaxHSV[k] = frameHSV[j, i][k]
    return (MinHSV, MaxHSV)


#THE FOLLOWING FUNCTIONS ARE EMPOWERED BY HIGHGUI


mouse_events = dict(map(lambda x:    (x, 0), range(0, 10)))
mouse_x      = 0
mouse_y      = 0
KEY_SPACE    = 32

def on_mouse_event(event, x, y, flags, params):
    global left_clicked
    global mouse_x, mouse_y
    mouse_events[event] = 1
    mouse_x = x
    mouse_y = y

def clear_events():
    keys = mouse_events.keys()
    for k in keys:
        mouse_events[k] = False

def GUIMinMaxHSV(img_in):
    if type(img_in) == type(''):
        frame = read_image(fname)
    else:
        frame = img_in
    cv2.namedWindow('wnd', 0)
    cv2.setMouseCallback('wnd', on_mouse_event, 0)
    cv2.imshow('wnd', frame)
    points_selected = 0
    points = []
    while cv2.waitKey(100) != 32:
        if len(points) == 2:
            print MinMaxHSV(frame, points[0][0], points[0][1], points[1][0], points[1][1])
            points = []
        if mouse_events[cv2.EVENT_LBUTTONDOWN]:
            points.append((mouse_x, mouse_y))
        clear_events()
    cv2.destroyWindow('wnd')
    cv2.waitKey(1000)

NO_INTERSECTION = (-1, -1)
def RangeIntersection(rng1, rng2):
    if rng2[0] > rng1[1] or rng1[0] > rng2[1]:
        return NO_INTERSECTION 
    else:
        if rng1[0] < rng2[0]:
            return (rng2[0], min(rng1[1], rng2[1]))
        else:
            return (rng1[0], min(rng1[1], rng2[1]))

def ColorRangeIntersection(rng1, rng2):
    ci11 = (rng1[0][0], rng1[1][0])
    ci12 = (rng1[0][1], rng1[1][1])
    ci13 = (rng1[0][2], rng1[1][2])

    ci21 = (rng2[0][0], rng2[1][0])
    ci22 = (rng2[0][1], rng2[1][1])
    ci23 = (rng2[0][2], rng2[1][2])

    return (RangeIntersection(ci11, ci21), \
            RangeIntersection(ci12, ci22), \
            RangeIntersection(ci13, ci23))

def ColorRangeIntersectionVolume(intrs):
    vol = int(inters[0][1] - inters[0][0] + 1) * \
          int(inters[1][1] - inters[1][0] + 1) * \
          int(inters[2][1] - inters[2][0] + 1)
    return vol

def get_cmass(img_in):
    img = read_image(img_in)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mom = cv2.moments(img_gray)
    mx = mom['m10'] / mom['m00']
    my = mom['m01'] / mom['m00']
    return (int(mx), int(my))

def hsv_filter_mask(hsv, hsv_range_low = np.array([0, 20, 32], dtype=np.uint8), hsv_range_high = np.array([180, 255, 240], dtype=np.uint8)):
    return cv2.inRange(hsv, hsv_range_low, hsv_range_high)

def test_color():
    color_ranges = []
    for color in ['red', 'blue', 'yellow']:
        img = read_image('../pure_markers/hand_3fingers_%s_frame300.png' % color)
        color_rng = MinMaxHSV(img, 0, 0, img.shape[1], img.shape[0], ignore_mask = np.array([255, 255, 255]))
        print color, color_rng
        color_ranges.append(color_rng)
    inters = ColorRangeIntersection(color_ranges[0], color_ranges[2])
    volume = ColorRangeIntersectionVolume(inters)
    print 'Intersection:', inters, 'Intersection volume:', volume 

def calc_back_proj(img, _hist, hsv = False, debug = False):
    if not hsv:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    back_proj = cv2.calcBackProject([img], channels = [0], \
                             hist = _hist['data'], \
                             ranges = [_hist['range'][0], _hist['range'][1]], \
                             scale = 1)
    if debug:
        max_val = np.max(back_proj)
        back_proj_uint8 = (back_proj / max_val * 256).astype('uint8')
        cv2.imwrite('calc_back_proj_debug.jpg', back_proj_uint8)
    return back_proj

def halt(msg):
    print '----!!!PROGRAM HALTED!!!----'
    print msg
    print '----------------------------'
    exit(1)

if __name__ == '__main__':
    #test_color()
    #print 'test_hue_hist:', test_hue_hist()
    #save_hist(h, IMAGE_FILE + '.hhst')
    #h = load_hist(IMAGE_FILE + '.hhst')
    #print h['data']
    #test_hist_save()
    #test_hist_show()
    #generate_hist_files(HIST_FILES)
    pass

def img_max(img):
    amax = np.argmax(img)
    max_x = amax % img.shape[1]
    max_y = amax / img.shape[1]
    return (max_x, max_y)

