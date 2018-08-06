#!/usr/bin/env python

#===================#
#      IMPORTS      #
#===================#

from time import time

#===================#
#    CONTROLLERS    #
#===================#

class P_Controller():
    def __init__(self, Kp):
        self.Kp = Kp

    def poll(self, error):
        return self.Kp * error

class PD_Controller():
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd

        self.previous_time = None
        self.previous_error = None

    def poll(self, error):
        p_term = self.Kp * error

        d_term = 0.0
        current_time = time()
        if self.previous_time is not None:
            d_term = self.Kd * (error - self.previous_error) / (current_time - self.previous_time)

        self.previous_time = current_time
        self.previous_error = error

        return p_term + d_term

class PID_Controller():
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.previous_time = None
        self.previous_error = None
        self.acc_error = 0.0

    def poll(self, error):
        current_time = time()
        p_term = self.Kp * error
        d_term = 0.0

        if self.previous_time is not None:
            delta_time = current_time - self.previous_time
            d_term = self.Kd * (error - self.previous_error) / delta_time
            self.acc_error += error * delta_time 

        self.previous_time = current_time
        self.previous_error = error

        return p_term + d_term + (self.Ki * self.acc_error)