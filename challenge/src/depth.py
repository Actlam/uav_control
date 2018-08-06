#!/usr/bin/env python

"""
This script uses the intel realsense camera to
try to locate obstacles in orer to inform our
finite state machine, for accurate z control.

It returns a Vector3 in the /error/obstacle topic:
msg.x = 0 if no obstacle, 1 if obstacle
msg.y = 0 if under, 1 if over
msg.z = Dist from the obstacle
"""

#===================#
#      IMPORTS      #
#===================#

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, Vector3

#===================#
#      CLASSES      #
#===================#

class DepthObstacleDetector:
    def __init__(self):
        rospy.loginfo("DepthObstacleDetector Started!")
        self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_cb)
        self.points_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.point_cb)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_cb)

        self.depth_pub = rospy.Publisher("/error/depth", Vector3, queue_size=1)

        self.bridge = CvBridge()
        self.points = []

    def local_pose_cb(self, msg):
        self.hover_height = msg.pose.position.z

    def point_cb(self, data):
        self.points = list(pc2.read_points(data))

    def depth_cb(self, imag):
        # First make sure we have a point cloud and height in the first place:
        if not (self.hover_height and self.points):
            rospy.loginfo("Waiting for hover height and point cloud...")
            return

        # Parse the image
        image = self.bridge.imgmsg_to_cv2(imag, "8UC1")

        # These are the eventual values to return.
        obstacle_found = False
        over = False
        obstacle_dist = 0.0

        # Apply morphology to find the barriers only.
        image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, np.ones((7, 13)))
        image = cv2.morphologyEx(image, cv2.MORPH_OPEN, np.ones((1,100)))

        # Contour the image, and sort the contours for h > 40
        _, cnts,_ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        acc = [brt for brt in (cv2.boundingRect(c) for c in cnts) if brt[3] > 40]

        if acc: # We've found some obstacles
            x, y, w, h = max(acc, key=lambda b: b[2]*b[3]) # Get the largest one

            y_bot = y+h # The bottom of the box
            x_mid = x + int(w/2) # And the center of it x-wise

            if isinstance(self.points[480*(y_bot-1)+x_mid][0], float):
                y, z = self.points[480*(y_bot-1)+x_mid][1:3]
                if y == y and z == z: # y,z is not NaN
                    h = self.hover_height - y # Get the height of the bottom edge 
                    if h > 0.15: # Not a ground thing:
                        obstacle_found = True
                        obstacle_dist = z
                        over = 0.15 < h < 1.05 # Go over if it's between 0.15 and 0.80

        if obstacle_found and over:
            rospy.loginfo("OVER OBSTACLE: {:.3f}m away, {:.3f}m high".format(obstacle_dist, h))
        elif obstacle_found and not over:
            rospy.loginfo("UNDER OBSTACLE: {:.3f}m away, {:.3f}m high".format(obstacle_dist, h))

        msg = Vector3()
        msg.x = 1.0 if obstacle_found else 0.0
        msg.y = 1.0 if over else 0.0
        msg.z = obstacle_dist
        self.depth_pub.publish(msg)

#===================#
#       MAIN        #
#===================#

if __name__ == '__main__':
    rospy.init_node('depth_obstacle_detector', anonymous=True)
    dod = DepthObstacleDetector()
    rospy.spin()

