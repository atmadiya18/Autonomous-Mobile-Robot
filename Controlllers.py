#!/usr/bin/env python
import numpy as np


class PidController:

    def __init__(self, kp, ki, kd, dt, outmin=-1, outmax=1): #self keeps track of pid terms
        self.err = 0
        self.integ = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.outmax = outmax
        self.outmin = outmin

    def step(self, err):

        deriv = (err - self.err) / self.dt # derivative based on identified dt
        integ = self.integ + err * self.dt # integral sum based on idenitfied dt

        out = self.kp * err + self.ki * integ + self.kd * deriv # pid equation
        #print("Command = ", out)
        out = np.clip(out, self.outmin, self.outmax) # clip function to not exceed min/max soft bounds

        self.err = err # store accumalted error
        self.integ = integ # store accumatled integer sum 

        return(out) # returns output speed