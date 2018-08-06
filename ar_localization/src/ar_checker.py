#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threadingn
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker


_Z_THRESH = 0.1

class ARDistChecker:
    def __init__(self):
        rospy.loginfo("ARDistChecker Started!")

        self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarker, self.ar_pose_cb)

        self.seen = set()
        self.current_marker = None

    def ar_pose_cb(self,msg):
        if len(msg.markers) < 1: 
            return

        marker = max(msg.markers, key=lambda x: x.confidence)
        self.current_marker = marker
        self.check_dist()

    def check_dist(self):
    	marker = self.current_marker
        z_des = self.get_dist(marker)

        if marker.id in self.seen:
            if time.time() - self.last_seen_message > 1:
                rospy.loginfo("SEEN: marker " + str(marker.id))
                self.last_seen_message = time.time()

        elif abs(z_des - 1) < _Z_THRESH:
            rospy.loginfo("CAPTURED: marker " + str(marker.id) + " OF " + str(len(self.seen)))
            self.seen.add(marker.id)
        else:
            if z_des > 1 + _Z_THRESH:
                rospy.loginfo("TOWARDS %.2fm closer" % (z_des - 1 - _Z_THRESH))
            else:
                rospy.loginfo("AWAY %.2fm away" % (1 + _Z_THRESH - z_des))

    def get_dist(self,mrk):
        return mrk.pose.pose.position.z

if __name__ == '__main__':
    rospy.init_node('ar_checker')
    a = ARDistChecker()

    rospy.spin()
