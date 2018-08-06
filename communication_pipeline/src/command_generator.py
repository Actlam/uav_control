#!/usr/bin/env python

from __future__ import division, print_function

import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist

'''
A python script to practice sending ROS messages
'''

class VelocityCommander():
    ''' Generates and publishes ROS messages
    '''
    def __init__(self, command_frequency=10.0):

        # publishing objects
        self.velocity_command_pub = rospy.Publisher("/velocity_command", Twist, queue_size=1)

        # rate of publishing
        self.command_frequency = rospy.Rate(command_frequency)

    def start_command_stream(self):
        ''' send velocity command messages
        '''
        while (not rospy.is_shutdown()):
            vel_cmd = Twist()
            t = time.time()
            vel_cmd.linear.x = 0.1 * np.sin(t * np.pi)
            vel_cmd.linear.y = 0 
            vel_cmd.linear.z = 0
            self.velocity_command_pub.publish(vel_cmd)
            self.command_frequency.sleep()

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('velocity_commander')
    vc_obj = VelocityCommander()
    print("VelocityCommander node running")

    # start the chatter
    vc_obj.start_command_stream()
    rospy.spin()
