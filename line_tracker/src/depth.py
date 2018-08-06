#!/usr/bin/env python

"""
This script uses the downwards camera for basic line following,
and returns the equation for the line in the /line/param topic
"""

#===================#
#      IMPORTS      #
#===================#

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
#===================#
#     CONSTANTS     #
#===================#

DEBUG = True
EXTRAPOLATED_DIST = 50

#===================#
#      HELPERS      #
#===================#

# 23-43
# 53-64

# So decided: bottom < 45in: over
# if bottom > 45in: under
#===================#
#      CLASSES      #
#===================#

class LineDetector:
    def __init__(self):
        rospy.loginfo("LineDetector Started!")
        self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_cb)
        self.points_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.point_cb)
        self.depth_pub = rospy.Publisher("/depth", Image, queue_size=1)
        self.bridge = CvBridge()
	self.points = []
	self.hover_height = .736
	#self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.loc_cb)

   # def loc_cb(self, data):
	#self.hover_height = data.pose.position.z

    def save_points(self, points):
        self.points = points

    def point_cb(self, data):
	points = []
	gen = pc2.read_points(data)
	for p in gen:
		points.append(p)
	self.save_points(points)
    
     
	
    def depth_cb(self, imag):
        image = self.bridge.imgmsg_to_cv2(imag, "8UC1")
        depth_image = self.bridge.imgmsg_to_cv2(imag, "16UC1")
        depth_array = np.array(depth_image, dtype=np.float32)
	obstacle_found = False
	over = False
	points = None
        #######################
        # Clarifying Obstacle #
        #######################
        image=cv2.morphologyEx(image, cv2.MORPH_CLOSE, np.ones((7, 13)))
        image= cv2.morphologyEx(image, cv2.MORPH_OPEN, np.ones((1,100)))

        xc = image.shape[1]/2
        yc = image.shape[0]/2
        cv2.circle(image, (xc, yc), 3, (255, 255, 255), thickness=3, lineType=8, shift=0) #makes center

	################################
        # Creating box around obstacle #
        ################################
        _, cnts,_ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	acc = [c for c in cnts if cv2.boundingRect(c)[3] > 40]
	def barrier_size(c):
	    _, _, w, h = cv2.boundingRect(c)
	    return w*h

	if acc:
	    obstacle_found = True # We have suitable obstacles.
	    cnt = max(acc, key=barrier_size)
	    x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,255),2)

            ybot = y+h
            xbot = x + int(w/2)
	    if self.points:
            	points = self.points
	    	if ((type(points[480*(ybot-1)+xbot][0]) == float) and (points[480*(ybot-1)+xbot][0] == points[480*(ybot-1)+xbot][0])):
	    	    x = points[480*(ybot-1)+xbot][0]
            	    y = points[480*(ybot-1)+xbot][1]
	    	    h_center_bot= y
	    	    #rospy.loginfo("x = {}".format(x))
	    	    #rospy.loginfo("y= {}".format(y))
	            #rospy.loginfo("h_center_bot = {}".format(h_center_bot))
		    if self.hover_height:
			rospy.loginfo("hover height = {}".format(self.hover_height))
			h = self.hover_height - h_center_bot
			rospy.loginfo("h = {}".format(h))
			if h < .80 and h > .15:
			    rospy.loginfo("OVER")
			elif h >= .8:
			    rospy.loginfo("UNDER")
			else:
			    rospy.loginfo("NONE")

	self.depth_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="8UC1"))
	self.points = []
        #################################
        # Deterine top, bot cord. depth #
        #################################





#===================#
#       MAIN        #
#===================#

if __name__ == '__main__':
    rospy.init_node('line_detector', anonymous=True)
    ld = LineDetector()
    rospy.loginfo("Line detector initialized")
    rospy.spin()

