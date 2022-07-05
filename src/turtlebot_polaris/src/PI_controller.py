#!/usr/bin/env python2

# 

import rospy

class PI(object):
    """ PI controller class
    
    Info:
    The PID controller continuously calculates an error and applies 
    a corrective action to resolve the error; in our case, the error is the motor 
    spinning at the wrong speed and the corrective action is changing the power to the motor. 
    It is this continuous testing of the motor speed and adjusting it to the correct speed 
    which will make your robot motors spin at the correct speed in order for it to orient 
    to the correct angle. For more infor, check https://github.com/laventura/PID-Control

    Attributes
    ----------
    kp : double
        Propotional Gain
    ki : double
        Integral Gain
    Current Time : double
        Present Time
    Previous Time : double
        Previous Time

    Methods
    -------
    clear()
        Resets the controller gains
    update(error)
        Calculates the control signal for the actuators
        Return : Control Signal (u)    
    """

    def __init__(self, Kp=0.01, Ki=0.0):

        self.kp = Kp     # Propotional Gain
        self.ki = Ki     # Integral Gain
        self.current_time = rospy.get_time()        # Current Time  
        self.previous_time = self.current_time      # Previous Time

        self.clear()    # reset the gain values

    def clear(self):
        self.P = 0.0       # Reset propotional gain
        self.I = 0.0       # Set integral gain to zero 

    def update(self, error):
        self.current_time = rospy.get_time()
        self.P = self.kp * error
        self.I += self.ki * error * (self.current_time - self.previous_time)
        #self.D_term = self.kd * ((error - self.previous_error)/ (self.current_time - self.previous_time))

        u = self.P + self.I 
        
        # update stored data for next iteration

        self.previous_time = self.current_time
        #self.previous_error = error
        return u