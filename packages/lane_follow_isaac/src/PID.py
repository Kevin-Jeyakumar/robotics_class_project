#!/usr/bin/env python3

class PID_Controller:
    def __init__(self):
        self.kp = 0.25
        self.ki = 0.01
        self.kd = 0.8

        self.last_time = 0
        self.last_error = 0
        self.i_term = 0

    def set_params(self, p, i, d):        
        self.kp = p
        self.ki = i
        self.kd = d

    def get_acc(self, error, time):
        p_term = error

        dt = time - self.last_time
        de = error - self.last_error

        if self.last_time > 0:
            self.i_term += error * dt
            d_term = de / dt
        else:
            self.i_term = 0
            d_term = 0

        self.last_time = time
        self.last_error = error

        a = (self.kp * p_term) + (self.ki * self.i_term) + (self.kd * d_term)
        return a
