#!/usr/bin/env python

"""
This is the main file, and the ONLY
one that will ever output velocity commands
to the drone, taking into account information
from all topics.
"""

#===================#
#      IMPORTS      #
#===================#

import rospy
from time import time
import threading
import numpy as np
from collections import deque
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3
from mavros_msgs.msg import State
from aero_control.msg import Line
from controllers import *

#===================#
#     CONSTANTS     #
#===================#

MAX_X = 1.0
MAX_Y = 1.0
MAX_Z = 1.5

#===================#
#    CONTROLLERS    #
#===================#

x_control   = P_Controller(0.012)
y_control   = P_Controller(0.008)
z_control   = P_Controller(1.5)
yaw_control = P_Controller(1.75)

#===================#
#      CLASSES      #
#===================#

class DroneController():
    """ Master Controller. Takes the errors from all the topics and
    converts them into desired velocity vectors to send to the drone
    in order to complete the basic challenge. """

    def __init__(self, hz=50):
        # The rate at which we publish our desired velocity.
        self.rate = rospy.Rate(hz)

        # Subscribe to gather errors + state, and publish velocity.
        self.line_error_sub = rospy.Subscriber("/error/line", Line, self.line_err_cb)
        self.z_error_sub = rospy.Subscriber("/error/z", Float64, self.z_err_cb)
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)

        # Setup asynchronous variables:
        self.x_err     = 0.0
        self.y_err     = 0.0
        self.theta_err = 0.0
        self.z_err     = 0.0

        self.vel_queue = deque([], 5) # Velocity commands will smoothen.

        # Asynchronous point streaming:
        self.state = None # MANUAL, POSCTL, OFFBOARD.
        self.start_streaming()

    # Callbacks:
    def line_err_cb(self, line):
        x, y, vx, vy = line.x, line.y, line.vx, line.vy
        self.x_err = (x - 64) # So that Kp can be positive.
        self.y_err = (64 - y) # So that Kp can be positive.

        try:
            self.theta_err = -np.arctan2(vx, vy)
        except ZeroDivisionError: # Vertical line
            self.theta_err = -90.0 if vy > 0 else 90.0

    def z_err_cb(self, z_err):
        self.z_err = -z_err.data # So that Kp can be positive.

    def state_cb(self, state):
        self.state = getattr(state, "mode", None)

    # Computing the velocity when asked by the streamer:
    def generate_vel(self):
        # Encode the latest velocity cmd:
        new_vel = [x_control.poll(self.x_err),
                   y_control.poll(self.y_err),
                   z_control.poll(self.z_err),
                   yaw_control.poll(self.theta_err)]
       
        # TURBO CODE:
        if abs(self.z_err) < np.pi/20: # If yaw error < 9 degrees:
            new_vel[0] = self.x_err * 0.025
            rospy.loginfo("TURBO")
        else:
            rospy.loginfo("NO TURBO")

        # Add it to the queue
        self.vel_queue.append(new_vel)

        # Get the average of the queue:
        avg_vel = [np.mean([v[x] for v in self.vel_queue]) for x in range(4)]

        # Now turn that average velocity into a TwistStamped:
        vel = TwistStamped()
        vel.twist.linear.x  = avg_vel[0]
        vel.twist.linear.y  = avg_vel[1]
        vel.twist.linear.z  = avg_vel[2]
        vel.twist.angular.z = avg_vel[3]
        return vel

    def start_streaming(self):
        def run_streaming():
            while not rospy.is_shutdown():
                if self.state != 'OFFBOARD':
                    vel = TwistStamped()
                    if self.vel_queue: # There are queued velocities from previous runs:
                        self.vel_queue.clear()
                else:
                    vel = self.generate_vel()

                if vel.twist.linear.x > MAX_X:
                    rospy.loginfo("CAPPING X VELOCITY FROM {} TO {}".format(vel.twist.linear.x, MAX_X))
                    vel.twist.linear.x = MAX_X

                if vel.twist.linear.y > MAX_Y:
                    rospy.loginfo("CAPPING Y VELOCITY FROM {} TO {}".format(vel.twist.linear.y, MAX_Y))
                    vel.twist.linear.y = MAX_Y

                if vel.twist.linear.z > MAX_Z:
                    rospy.loginfo("CAPPING Z VELOCITY FROM {} TO {}".format(vel.twist.linear.z, MAX_Z))
                    vel.twist.linear.z = MAX_Z

                self.vel_pub.publish(vel)
                self.rate.sleep()

        streaming_thread = threading.Thread(target=run_streaming)
        streaming_thread.start()

#===================#
#       MAIN        #
#===================#

if __name__ == '__main__':
    rospy.init_node('drone_controller', anonymous=True)
    ld = DroneController()
    rospy.loginfo("MASTER drone controller initialized")
    rospy.spin()
