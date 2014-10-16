from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2

CAMSHIFT_CRITERION_DEF = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

class CamShift:
    criterion = CAMSHIFT_CRITERION_DEF

    def __init__(self):
        self.prev_track_box = None
    
    def init(self, img, blobs):
        #first find the initial window position
        self.prev_track_box = []
        for i in range(len(markers)):
            self.prev_track_box.append(())
        for m in markers:
            if m.hist:
                hs = m.hist
                bkproj = calc_back_proj(img, hs.hist)
                max_arg = img_max(bkproj)
                #TODO: the actual initial window search
                #show_image(bkproj, raw = True, show_max = True)
                (m.center.x, m.center.y) = max_arg
            else:
                c = m.center
                s = m.size
                w = (c.x - s.x / 2, c.y - s.y / 2, c.x + s.x / 2, c.y + s.y / 2)
                img_roi = img[w[1]:w[3], w[0]:w[2]]
                hst = Histogram()
                hst.calc(img_roi)
                m.hist = hst

    #WARNING: @run alters @img
    def run(self, img, markers):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = hsv_filter_mask(img_hsv)
        next_windows = []
        prob_imgs = []
        markers_hue = []
        #for each histogram from histogrs
        for i in range(len(markers)):
            m = markers[i]
            c = m.center
            s = m.size
            w = (c.x - s.x / 2, c.y - s.y / 2, s.x, s.y)
            back_proj = calc_back_proj(img_hsv, m.hist.hist, hsv = True)
            back_proj &= mask
            #cv2.imwrite('debug.jpg', back_proj)
            #print 'IN:', w
            (track_box, new_window) = cv2.CamShift(back_proj, w, self.criterion)
            #print 'CamShift OUT:', track_box, new_window
            #update markers' pos, but only if track_box is valid
            if track_box[1][0] != 0.0 and track_box[1][1] != 0.0:
                self.prev_track_box[i] = track_box
                c = Point(new_window[0] + new_window[2] / 2, \
                          new_window[1] + new_window[3] / 2, 0)
                s = Point(new_window[2] if new_window[2] <= \
                                             m.limit.x \
                                        else m.limit.x, \
                          new_window[3] if new_window[3] <= \
                                             m.limit.y \
                                        else m.limit.y, 0)
                m.center = c
                m.size = s
            else:
                c = m.center
                s = m.size
            try:
                markers_hue.append(int(img_hsv[c.y][c.x][0]))
            except IndexError, e:
                print 'OOPS!!!'
                cv2.imwrite('dd.png', img)
            #markers_hue.append(int(back_proj[c.y][c.x]))
            #print markers_hue[len(markers_hue)-1]
            back_proj = cv2.cvtColor(back_proj, cv2.COLOR_GRAY2BGR)#ACHTUNG: back_proj not mono anymore
            track_box = self.prev_track_box[i]
            if track_box != ():
                cv2.ellipse(back_proj, track_box, RGB(255, 0, 0), 1)
            prob_imgs.append(back_proj)
            #draw debug info
            #TODO: display information about several markers, not only one
            if track_box != ():
                mean_val = (int(track_box[0][0]), int(track_box[0][1]))
                info_msgs = ['Marker mode: (X, Y) = (%d, %d)' % \
                      (mean_val[0], \
                       mean_val[1]), \
                      'Bounding box area: %d' % \
                       (track_box[1][0] * track_box[1][1])]
                draw_label(img, str(i), mean_val)
                draw_cross(img, mean_val, length = 20, \
                           color = RGB(0, 255, 0), thickness = 1)
                if m.id != 'Object':
                    cv2.ellipse(img, track_box, RGB(255, 255, 255), 1)
                else:
                    cv2.ellipse(img, track_box, RGB(255, 0, 0), 1)
                draw_debug_messages(img, info_msgs)
        #draw_bound_rectangle(img, points, 0, 255)
        return (img, prob_imgs, markers_hue)

class TrackerRGBD:
    def __init__(self):
        self.target = Point()

    def init(self, target_point):
        self.target = target_point

    def feed_rgb(self, Image):
        pass

    def feed_depth(self, Image):
        pass
