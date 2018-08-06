#!/usr/bin/env python

"""
This file detects any objects in the way and plans
a flight path through them when necessary. This is
imported into the state_manager when needed.
"""

#===================#
#      IMPORTS      #
#===================#

import numpy as np
import mavros
import rospy
import threading
from time import time
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped

#===================#
#     CONSTANTS     #
#===================#

LINE_HOVER_HEIGHT = 0.7 # Height you should hover at, in meters
OBSTACLE_RESPONSE_RANGE = 1.5 # respond to artags within 1.5 meters

#===================#
#      CLASSES      #
#===================#

class ObstaclePathPlanner():
    """ Recieves ARTag information + combines w/ intel realsense camera
    to return the desired z height. Gets sent to the PIDs in the state
    manager to eventually reach the height we want. This needs to have
    it's own internal state machine, to capture the fact that it's either
    pathfinding around an object, or is simply keeping x at it's default
    line - following value. """

    def __init__(self, hz=50):

        rospy.loginfo("ObstaclePathPlanner Started!")

        mavros.set_namespace()

        self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_cb)
        self.z_error_pub = rospy.Publisher("/error/z", Float64, queue_size=1)

        """ State:
        0: Hovering in place
        1: Line following
        2: Going over obstacles
        3: Going under obstacles
        """

        self.state = 1
        self.rate = rospy.Rate(hz)
        self.hz = hz

        # Async variables:
        self.markers = []
        self.current_pose = None

        # Vars for states 2 & 3:
        self.current_obstacle = None
        self.current_obstacle_dist = None
        self.artag_seen_x = None

        self.send_state()

    def ar_pose_cb(self, msg):
        self.markers = msg.markers

    def local_pose_cb(self, msg):
        self.current_pose = msg

    def update_state(self):
        if self.state == 0: return # If hovering, keep hovering.

        if self.state > 1: # If you're going over/under a hurdle:
            if abs(self.artag_seen_x - self.current_pose.pose.position.x) > self.current_obstacle_dist + 0.3:
                self.state = 1
                rospy.loginfo("CHANGING STATE TO 1: LINE FOLLOW")

        if self.markers: # If there are markers:
            candidate = min(self.markers, key=lambda m:m.pose.pose.position.z) # find nearest tag
            if candidate.pose.pose.position.z < OBSTACLE_RESPONSE_RANGE: # If it's within range
                self.current_obstacle = candidate # Set that as your target
                if candidate.id % 2: # If it's odd
                    self.state = 2 # You're going over it
                    rospy.loginfo("GOING OVER OBSTACLE {}".format(candidate.id))
                elif candidate.id > 0 and candidate.id % 2 == 0: # but if it's even
                    self.state = 3 # You're going under it
                    rospy.loginfo("GOING UNDER OBSTACLE {}".format(candidate.id))

    def artag_hurdle_height(self, over):
        """ Given {bool} over, uses the ARTAG to determine z error. """
        if any(x.id == self.current_obstacle.id for x in self.markers):
            my_obstacle_artag = [x for x in self.markers if x.id == self.current_obstacle.id][0]
            self.current_obstacle_dist = my_obstacle_artag.pose.pose.position.z
            self.artag_seen_x = self.current_pose.pose.position.x

        if over:
            return self.current_pose.pose.position.z - 1.2 # 1.2m for over an obstacle
        else:
            return self.current_pose.pose.position.z - 0.6 # 0.5m under an obstacle

    def send_state(self):
        """ Based on the state, does calculations, and sends it
        to the /error/z topic """
        def run_streaming():
            while self.current_pose is None:
                pass # Wait until we recieve the pose.

            while not rospy.is_shutdown():
                self.update_state()

                if self.state == 0: # Hover:
                    self.z_error_pub.publish(0.0)
                elif self.state == 1: # Line following
                    self.z_error_pub.publish(self.current_pose.pose.position.z - LINE_HOVER_HEIGHT)
                else: # Dealing with hurdles (state of 2 is over, state of 3 is under.
                    self.z_error_pub.publish(self.artag_hurdle_height(self.state == 2))

                self.rate.sleep()

        threading.Thread(target=run_streaming).start()

#===================#
#        MAIN       #
#===================#

if __name__ == '__main__':
    rospy.init_node('obstacle_planner')
    a = ObstaclePathPlanner()
    rospy.spin()


