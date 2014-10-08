from sensor_msgs.msg import Image
from geometry_msgs.msg import Point


class CamshiftRGBD:
    def __init__(self):
        self.target = Point()

    def init(self, target_point):
        self.target = target_point

    def feed_rgb(self, Image):
        pass

    def feed_depth(self, Image):
        pass
