#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threading
import numpy as np
import tf
from tf.transformations import *
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, PoseArray
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from std_msgs.msg import String


import mavros
from mavros_msgs.msg import State

_DIST_TO_OBST = {24:[1.1, 0.25, 0.0, 0.0],12:[0.0,0.0,0.0,0.0], 9:[0.0, 0.0, 0.0, 0.0]} 

_OBST_SEQ = [24]

_YAW_DES = 0.0 # radians

# P(ID) constants
_K_P_X = 0.2
_K_P_Y = -0.2
_K_P_Z = -0.2

_K_P_YAW = 0.5

_DEBUG =True

class ARObstacleController:
    def __init__(self, hz=60):
        rospy.loginfo("ARObstacleController Started!")
        mavros.set_namespace()
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_cb)
        self.local_pose_sp_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.local_vel_sp_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)

        self.ar_pose_sub = rospy.Subscriber("/ar_aero_pose", AlvarMarkers, self.ar_pose_cb)

        self.tag_error = rospy.Publisher("/tag_error", Twist, queue_size=1)

        self.obstacles = {12 : 3, 24 : 2, 9: 4} # dict (marker -> mode)
        self.rate = rospy.Rate(hz)
        self.current_state = State()
        self.current_pose = None
        self.current_vel = None

        '''
         0 is hovering in space, 
         1 is flying to obstacle 
         2 is ring flythru (marker id: 24)
         3 is hurdle flyover (marker id: 12)
         4 is gate flyunder (marker id: )
        '''
        self.finite_state = 0 
        self.markers = []
        self.vel_hist = [[],[],[],[]]
        self.current_obstacle_seq = 0
        self.current_obstacle_tag = None
        self.t_marker_last_seen = None
        self.t_obstacle_start = None

        self.local_vel_sp = TwistStamped()
        self.local_pose_sp = None

        self.offboard_vel_streaming = False

        self.tl = tf.TransformListener()

    def state_cb(self, msg):
        self.current_state = msg

    def ar_pose_cb(self,msg):
        self.markers = msg.markers
    
        self.t_marker_last_seen = datetime.now()

        self.update_finite_state()

    def update_finite_state(self, mode=0, force=False): # updates current phase of avoidance 
        if force:
            self.finite_state = mode
            return


        if self.t_marker_last_seen is not None and self.finite_state < 2:
            self.td = datetime.now() - self.t_marker_last_seen
            if self.td.total_seconds() > 1: 
                self.finite_state = 0 # Just Hover

        if mode != 0:
            self.finite_state = mode

        if any(marker.id in self.obstacles for marker in self.markers) and self.finite_state == 0:
            self.current_obstacle_tag = max(self.markers, key=lambda m:m.confidence).id
            self.finite_state = 1
            
    def generate_vel(self): # assesses course of action using finite states
        if self.finite_state == 0:
            self.vel_hist[0].insert(0,0)
            self.vel_hist[1].insert(0,0)
            self.vel_hist[2].insert(0,0)
            self.vel_hist[3].insert(0,0)

        elif self.finite_state == 1:
            self.fly_to_obstacle()

        elif self.finite_state == 2:
            rospy.logerr("avoiding ring")
            self.current_obstacle_tag = 24
            if self.t_obstacle_start == None:
                self.clear_history(wipe=True) 
                self.t_obstacle_start = datetime.now()
            self.avoid_ring()

        elif self.finite_state == 3:
            rospy.logerr("avoiding hurdle")
            self.current_obstacle_tag = 12
            if self.t_obstacle_start == None:
                self.clear_history(wipe=True) 
                self.t_obstacle_start = datetime.now()

            self.avoid_hurdle()

        elif self.finite_state == 4:
            self.current_obstacle_tag = 9
            if self.t_obstacle_start == None:
                self.clear_history(wipe=True) 
                self.t_obstacle_start = datetime.now()
            self.avoid_gate()

        self.smooth_vel()

        vel = self.local_vel_sp
        if _DEBUG: rospy.loginfo("vel cmd: x: " + "%.05f" % vel.twist.linear.x + " y: " + "%.05f" % vel.twist.linear.y + " z: " + "%.05f" % vel.twist.linear.z + " yaw: " + "%.05f" % vel.twist.angular.z)

    def fly_to_obstacle(self): # once an AR tag is detected, fly to that obstacle to prepare for avoidance
        marker_list = [marker for marker in self.markers if marker.id in self.obstacles]
        if len(marker_list) < 1: return
        target_marker = min(marker_list, \
            key=lambda marker: marker.pose.pose.position.x)

        if target_marker is None: 
            self.update_finite_state()
            rospy.loginfo("no obstacles found!")
            return

        if target_marker.id != _OBST_SEQ[self.current_obstacle_seq]:
            rospy.logerr("wrong obstacle detected!! quitting for safety")
            return

        curr_yaw = self.get_yaw(target_marker.pose.pose.orientation)
        curr_x = target_marker.pose.pose.position.x
        curr_y = target_marker.pose.pose.position.y
        curr_z = target_marker.pose.pose.position.z

<<<<<<< HEAD
        x_error = curr_x - _DIST_TO_OBST[target_marker.id][0]
        y_error = curr_y - _DIST_TO_OBST[target_marker.id][1]
        z_error = curr_z - _DIST_TO_OBST[target_marker.id][2]
        yaw_error = curr_yaw - _YAW_DES + (np.pi/2)
=======
        x_error = None 
        y_error = None
        z_error = None
        yaw_error = None
        raise Exception("calculate errors and delete this!!")
>>>>>>> 6e42488d85009858d2c507af9da6e4e039156e86

        if _DEBUG: rospy.loginfo("error: x: %.04f y: %.04f z: %.04f yaw %.04f" % (x_error, y_error, z_error, yaw_error))
        msg = Twist()
        msg.linear.x = x_error
        msg.linear.y = y_error
        msg.linear.z = z_error
        msg.angular.z = yaw_error
        self.tag_error.publish(msg)

        if abs(x_error) < 0.1 and abs(y_error) < 0.1 and abs(z_error) < 0.1: # we can start flying thru
            # above thresholds (0.1 for all currently) are modifiable!!
            self.update_finite_state(self.obstacles[target_marker.id])
            self.current_obstacle_seq+= 1 if self.current_obstacle_seq < len(_OBST_SEQ) else 0
            return

        self.vel_hist[0].insert(0,x_error*_K_P_X)
        self.vel_hist[1].insert(0,y_error*_K_P_Y)
        self.vel_hist[2].insert(0,z_error*_K_P_Z)
        self.vel_hist[3].insert(0,yaw_error*_K_P_YAW)

    def avoid_ring(self): # commands vel such that ring can be avoided open-loop
        td = datetime.now()-self.t_obstacle_start

###########################################################################################################################
# TODO: decide how long / at what vel to go up/forward to avoid ring
###########################################################################################################################

        t_up = 2.0
        t_forward = 3.0

        if td.total_seconds() < t_up:
            self.vel_hist[2].insert(0, 0.4)
            rospy.loginfo("ring avoid: going up!")

        elif td.total_seconds() < t_forward and td.total_seconds() > t_up:
            self.clear_history(z=True)
            self.vel_hist[0].insert(0, 0.40)
            rospy.loginfo("ring avoid: going forward!")

        else:
            self.clear_history(x=True, z=True)
            self.t_obstacle_start = None
            self.update_finite_state(force=True)
    def avoid_hurdle(self): # commands vel such that hurdle can be avoided open-loop
        td = datetime.now()-self.t_obstacle_start

###########################################################################################################################
# TODO: decide how long / at what vel to go up/forward to avoid hurdle
###########################################################################################################################
        t_up = None
        t_forward = None

        # raise Exception("hurdle avoid times!")
        if td.total_seconds() < t_up:
            # add to vel_hist here!! (insert at zero)
            if _DEBUG: rospy.loginfo("hurdle avoid: going up!")
        elif td.total_seconds() < t_forward and td.total_seconds() > t_up:
            self.clear_history(z=True)
            # add to vel_hist here!! (insert at zero)
            if _DEBUG: rospy.loginfo("hurdle avoid: going forward!")
        else:
            self.clear_history(x=True, z=True)
            self.t_obstacle_start = None
            self.update_finite_state(force=True)

    def avoid_gate(self): # commands vel such that gate can be avoided open-loop
        td = datetime.now()-self.t_obstacle_start

###########################################################################################################################
# TODO: decide how long / at what vel to go down/forward to avoid gate
###########################################################################################################################
        t_down = None
        t_forward = None

        # raise Exception("gate avoid times!")
        if td.total_seconds() < 0.5:
            # add to vel_hist here (insert at zero)
            if _DEBUG: rospy.loginfo("gate avoid: going down!")
        elif td.total_seconds() < 7 and td.total_seconds() > 0.5:
            self.clear_history(z=True)
            # add to vel_hist here (insert at zero)
            if _DEBUG: rospy.loginfo("gate avoid: going forward")
        else:
            self.clear_history(x=True, z=True)
            self.t_obstacle_start = None
            self.update_finite_state(force=True)


    def smooth_vel(self): # running average to produce smoother movements 
        for i in range(len(self.vel_hist)-1):
            if len(self.vel_hist[i]) > 19:
                self.vel_hist[i].pop()

        if len(self.vel_hist[3]) > 5:
            self.vel_hist[3].pop()
    
    
        x_vel = sum(self.vel_hist[0])/len(self.vel_hist[0]) if len(self.vel_hist[0])>0 else 0

        y_vel = sum(self.vel_hist[1])/len(self.vel_hist[1]) if len(self.vel_hist[1])>0 else 0

        z_vel = sum(self.vel_hist[2])/len(self.vel_hist[2]) if len(self.vel_hist[2])>0 else 0

        yaw_vel = sum(self.vel_hist[3])/len(self.vel_hist[3]) if len(self.vel_hist[3]) > 0 else 0


        self.local_vel_sp.twist.linear.x = x_vel
        self.local_vel_sp.twist.linear.y = y_vel
        self.local_vel_sp.twist.linear.z = z_vel
        self.local_vel_sp.twist.angular.z = yaw_vel


    def get_yaw(self,quat_msg): # converts orientation quaternion to euler and gets yaw
        q_a_wenu = [quat_msg.x,quat_msg.y,quat_msg.z,quat_msg.w]

        return tf.transformations.euler_from_quaternion(q_a_wenu)[2] # yaw


    def clear_history(self,x=False, y=False, z=False, yaw=False, wipe=False): # clears vel hist (helpful at transitions)
        if wipe:
            self.vel_hist = [[],[],[],[]]
            return
        if x:
            self.vel_hist[0] = [0 for i in self.vel_hist[0]]
        if y:
            self.vel_hist[1] = [0 for i in self.vel_hist[1]]
        if z:
            self.vel_hist[2] = [0 for i in self.vel_hist[2]]
        if yaw:
            self.vel_hist[3] = [0 for i in self.vel_hist[3]]

###########################################################################################################################
# DO NOT MODIFY BELOW THIS COMMENT
###########################################################################################################################
               
    def start_streaming_offboard_vel(self):
        def run_streaming():
            self.offboard_vel_streaming = True
            while not rospy.is_shutdown() and self.current_state.mode != 'OFFBOARD':
        
        # Publish a "don't move" velocity command
                velocity_message = TwistStamped()
                self.local_vel_sp_pub.publish(velocity_message)
                rospy.loginfo('Waiting to enter offboard mode')
                rospy.Rate(60).sleep()

        # Publish at the desired rate
            while (not rospy.is_shutdown()) and self.offboard_vel_streaming:
                self.update_finite_state()
                vel = TwistStamped()
                self.generate_vel()
            # create a vel setpoint based on the vel setpoint member variable
                vel = self.local_vel_sp
           
            # Create a zero-velocity setpoint
            # vel = Twist()    
                self.local_vel_sp_pub.publish(vel)
                self.rate.sleep()

        self_offboard_vel_streaming_thread = threading.Thread(target=run_streaming)
        self_offboard_vel_streaming_thread.start()


    def stop_streaming_offboard_vel(self):
        self.offboard_vel_streaming = False
        try:
            self_offboard_vel_streaming_thread.join()
        except AttributeError:
            pass

    def local_pose_cb(self,msg):
        self.current_pose = msg

if __name__ == '__main__':
    rospy.init_node('ar_obstacle')
    a = ARObstacleController()

    a.start_streaming_offboard_vel()

    rospy.spin()