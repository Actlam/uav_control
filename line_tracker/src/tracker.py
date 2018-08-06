#!/usr/bin/env python

#===================#
#      IMPORTS      #
#===================#

from __future__ import division, print_function
import rospy, mavros, cv2, threading
import numpy as np
from aero_control.msg import Line
from mavros_msgs.msg import State
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion, Point, Vector3
from tf.transformations import quaternion_from_euler, quaternion_matrix
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy
from controllers import *

#===================#
#     CONSTANTS     #
#===================#

NO_ROBOT = True # set to True to test on laptop
MAX_SPEED = 0.5 # [m/s]
IMG_CENTER = (64, 64)

# X Controller:
Kp_x = 0.020
Kd_x = 0.007

# Y Controller:
Kp_y = 0.0175
Kd_y = 0.000

# Yaw controller:
Kp_yaw = -1.0
Kd_yaw = 0.000

#===================#
#      CLASSES      #
#===================#

class LineTracker:
    def __init__(self, rate=10):
        """ Initializes publishers and subscribers, sets initial values for vars
        :param rate: the rate at which the setpoint_velocity is published
        """
        assert rate > 2 # make sure rate is high enough, if new setpoint recieved within .5 seconds, robot will switch back to POSCTL
        self.rate = rospy.Rate(rate)

        mavros.set_namespace()
        self.bridge = CvBridge()

        self.pub_local_velocity_setpoint = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
        self.sub_line_param = rospy.Subscriber("/line/param", Line, self.line_param_cb)
        self.pub_error = rospy.Publisher("/line/error", Vector3, queue_size=1)


        # Variables dealing with publishing setpoint
        self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.current_state = None
        self.offboard_point_streaming = False

        # Setpoint field expressed as the desired velocity of the body-down frame
        #  with respect to the world frame parameterized in the body-down frame
        self.velocity_setpoint = None

        # Create my controllers:
        self.x_control = PD_Controller(Kp_x, Kd_x)
        self.y_control = PD_Controller(Kp_y, Kd_y)
        self.yaw_control = PD_Controller(Kp_yaw, Kd_yaw)

        while not rospy.is_shutdown() and self.current_state == None: pass  # Wait for connection

    def line_param_cb(self, line_params):
        mode = getattr(self.current_state, "mode", None)
        if (mode is not None and mode != "MANUAL") or NO_ROBOT:
            x, y, vx, vy = line_params.x, line_params.y, line_params.vx, line_params.vy

            error_x = x - IMG_CENTER[0]
            error_y = y - IMG_CENTER[1]

            try:
                m = vy/vx
                error_yaw = np.arctan2(vx,vy)
            except ZeroDivisionError: # Vertical line
                error_yaw = -90.0 if vy > 0 else 90.0

            self.pub_error.publish(Vector3(error_x, error_y, error_yaw))

            x_vel = self.x_control.poll(error_x)
            y_vel = self.y_control.poll(error_y)
            yaw_vel = self.yaw_control.poll(error_yaw)

            self.velocity_setpoint = TwistStamped()
            self.velocity_setpoint.twist.linear.x = x_vel
            self.velocity_setpoint.twist.linear.y = -y_vel
            self.velocity_setpoint.twist.angular.z = yaw_vel


    def state_cb(self, state):
        """ Starts setpoint streamer when mode is "POSCTL" and disables it when mode is "MANUAL"
        :param state: Given by subscribed topic `/mavros/state`
        """
        self.current_state = state
        mode = getattr(state, "mode", None)
        if (mode == "POSCTL" or NO_ROBOT) and not self.offboard_point_streaming:
            rospy.loginfo("Setpoint stream ENABLED")
            self.start_streaming_offboard_points()
        elif mode == "MANUAL" and self.offboard_point_streaming:
            rospy.loginfo("Setpoint stream DISABLED")
            self.stop_streaming_offboard_points()

    def start_streaming_offboard_points(self):
        """ Starts thread that will publish yawrate at `rate` in Hz
        """
        def run_streaming():
            self.offboard_point_streaming = True
            while (not rospy.is_shutdown()) and self.offboard_point_streaming:
                # Publish commands
                if (self.velocity_setpoint is not None):
                    # limit speed for safety
                    velocity_setpoint_limited = deepcopy(self.velocity_setpoint)
                    speed = np.linalg.norm([velocity_setpoint_limited.twist.linear.x,
                                            velocity_setpoint_limited.twist.linear.y,
                                            velocity_setpoint_limited.twist.linear.z])
                    if speed > MAX_SPEED:
                        velocity_setpoint_limited.twist.linear.x *= MAX_SPEED / speed
                        velocity_setpoint_limited.twist.linear.y *= MAX_SPEED / speed
                        velocity_setpoint_limited.twist.linear.z *= MAX_SPEED / speed

                    # Publish limited setpoint
                    self.pub_local_velocity_setpoint.publish(velocity_setpoint_limited)
                self.rate.sleep()

        self.offboard_point_streaming_thread = threading.Thread(target=run_streaming)
        self.offboard_point_streaming_thread.start()

    def stop_streaming_offboard_points(self):
        """ Safely terminates offboard publisher
        """
        self.offboard_point_streaming = False
        try:
            self.offboard_point_streaming_thread.join()
        except AttributeError:
            pass

#==================#
#       MAIN       #
#==================#

if __name__ == "__main__":
    rospy.init_node("line_tracker")
    rospy.loginfo("Line tracker initialized")
    d = LineTracker()
    rospy.spin()
d.stop_streaming_offboard_points()