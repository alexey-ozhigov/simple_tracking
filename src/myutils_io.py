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




#/usr/bin/env python

import cv2
import cv
from cv2.cv import CV_CAP_PROP_FRAME_COUNT, CV_CAP_PROP_FPS, \
                   CV_CAP_PROP_FRAME_WIDTH, CV_CAP_PROP_FRAME_HEIGHT 
import numpy as np

def read_image(img_in):
    if type(img_in) == type(''):
        frame = cv2.imread(img_in)
        assert frame != None
    else:
        frame = img_in
        assert frame != None
        assert type(frame) == type(np.array([]))
    return frame

def open_video2(in_file):
    capt = cv2.VideoCapture(in_file)
    frames_num = int(capt.get(CV_CAP_PROP_FRAME_COUNT))
    fps = int(capt.get(CV_CAP_PROP_FPS))
    frame_width = int(capt.get(CV_CAP_PROP_FRAME_WIDTH))
    frame_height = int(capt.get(CV_CAP_PROP_FRAME_HEIGHT))
    return (capt, frames_num, fps, frame_width, frame_height)

def read_image_from_video(video, offset = 0):
    (capt, frames_num, _, _, _) = open_video2(video)
    if not capt:       
        raise 'Cannot open video', in_file
    if offset >= frames_num:
        raise 'Cannot read frame offset %d from' % offset, video
    for i in range(0, offset):
        capt.read()
    return capt.read()

def read_images_from_video(video, offset = 0, length = 0):
    (capt, frames_num, _, _, _) = open_video2(video)
    if length == 0:
        length = frames_num - offset
    if not capt:       
        raise 'Cannot open video', in_file
    if offset >= frames_num:
        raise Exception('Cannot read frame offset %d from %s' % (offset, video))
    for i in range(0, offset):
        capt.read()
    frames = []
    for i in range(length):
        frames.append(capt.read())
    return frames


def copy_paste_video(video_in, video_out, start_frame = 0, total_frames = 0):
    (capt, frames_num, fps, frame_width, frame_height) = open_video2(video_in)
    vwriter = cv2.VideoWriter(video_out, cv.CV_FOURCC('P', 'I', 'M', '1'), \
                                   fps, (frame_width, frame_height), True)
    assert start_frame >= 0
    if total_frames == 0:
        total_frames = frames_num - start_frame
    if start_frame + total_frames > frames_num:
        total_frame = frames_num - start_frame
    
    print 'Copy pasting frames %d-%d (total %d) from %s to %s' % \
            (start_frame, start_frame + total_frames, total_frames, video_in, video_out)
    
    frame_i = 0
    while frame_i < start_frame:
        _ = capt.read()
        frame_i += 1
        #print 'Skipped frame %d out of %d' % (frame_i, start_frame)

    frame_i = 0
    while frame_i < total_frames:
        frame = capt.read()
        if frame[0]:
            vwriter.write(frame[1])
            #print 'Written frame %d out of %d' % (frame_i, total_frames)
        frame_i += 1

def insert_pause_to_video(vwriter, pause_frame, num = 30):
    for i in range(0, num):
        vwriter.write(pause_frame)

if __name__ == '__main__':
    copy_paste_video('hand_depth.avi', '1_depth.avi', 1978, 258)
    copy_paste_video('hand_rgb.avi', '1_rgb.avi', 2004, 258)
