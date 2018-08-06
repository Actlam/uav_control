#!/usr/bin/env python

#===================#
#      IMPORTS      #
#===================#

import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from aero_control.msg import Line
from scipy.stats import linregress

#===================#
#     CONSTANTS     #
#===================#

DEBUG = True
EXTRAPOLATED_DIST = 50


#===================#
#      HELPERS      #
#===================#

def find_inliers(m,b,shape):
    for x1 in range(shape[0]):
        y1 = m * x1 + b
        if 0 <= y1 <= shape[1]:
            break
    
    for x2 in reversed(range(shape[0])):
        y2 = m * x2 + b
        if 0 <= y2 <= shape[1]:
            break

    # NaN != NaN, so this effectively converts NaNs to zeros.
    return [int(x) if x==x else 0 for x in (x1, y1, x2, y2)]

#===================#
#      CLASSES      #
#===================#

class LineDetector:
    def __init__(self):
        self.sub_cam = rospy.Subscriber("/aero_downward_camera/image", Image, self.image_cb)
        self.pub_param = rospy.Publisher("/line/param", Line, queue_size=1)
        self.image_pub = rospy.Publisher("/image_to_cv/processed", Image, queue_size=1)
        
        self.bridge = CvBridge()

    def segmentLine(self, cv_image):
        """ Fit a line to LED strip
        :param cv_image: opencv image
        """

        # Filter the image
        filtered = cv2.filter2D(cv_image, -1, np.ones((3,3))/9)
        filtered = cv2.inRange(filtered, 245, 255)
        aw = np.argwhere(filtered)

        # Apply Linear Regression.
        if aw.shape[0]:
            m, b, _, _, _ = linregress(aw[:,1], aw[:,0])
            x1, y1, x2, y2 = find_inliers(m, b, cv_image.shape)

            # Compute the extrapolated point:
            img_center = (cv_image.shape[0]/2, cv_image.shape[1]/2)
            line_close_x = (img_center[0] + (m*(img_center[1]-b))) / ((m*m)+1)
            line_close = tuple(int(x) if x==x else 0 for x in (line_close_x, m*line_close_x + b))
            vx, vy = 1.0/np.sqrt((m*m) + 1), m/np.sqrt((m*m) + 1)
            extrapolated = (vx*EXTRAPOLATED_DIST + line_close[0], vy*EXTRAPOLATED_DIST + line_close[1])
            extrapolated = tuple(int(x) if x==x else 0 for x in extrapolated)

        # Display the image, for debugging:
        if DEBUG:
            orig = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB) # Keep a color version of the image
            if aw.shape[0]:
                cv2.line(orig, (x1,y1), (x2,y2), (0, 0, 255), 2)
                cv2.line(orig, extrapolated, img_center, (255, 0, 255), 1)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(orig, "rgb8"))

        # Publish the information
        if aw.shape[0]:
            return extrapolated[0], extrapolated[1], vx, vy

    def image_cb(self, data):
        line = self.segmentLine(self.bridge.imgmsg_to_cv2(data, "8UC1"))
        if line is not None:
            x, y, vx, vy = line
            msg = Line()
            msg.x = x
            msg.y = y
            msg.vy = vx
            msg.vx = vy
            self.pub_param.publish(msg)

#===================#
#       MAIN        #
#===================#

if __name__ == '__main__':
    rospy.init_node('line_detector', anonymous=True)
    ld = LineDetector()
    rospy.loginfo("Line detector initialized")
    rospy.spin()
