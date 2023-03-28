#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 27 03:58:59 2023

@author/s: 
PID Speed Controller: jingxuejiang
Stanley Steering Controller: 
GPS wayPoints transformation: 

@reference

@ IMPORTANT: FOLLOWING CASES NEED TO BE TESTED BEFORE DEPLOYMENT 
1) zero reference speed. ref = 0 
2) negative reference speed for P term and I term seperately
3) when reference speed == desired speed 
4) Test output limit block 
5) Test if back-calculation term is working correctly. (dt / self.Tt ) * (u - u_temp); 
6) Integral term should reset when reference signal changes values 
7) test the controller with given dt 

"""

# Import python packages
import time 
# Import ROS headers
# import rospy

# Import Novetal GNSS unit headers

# Class for PID controller

class PID(object):
    
    def __init__(self,k,Ti,Tt,b,u_min = None,u_max = None):
        
        """
        k: gain. (strength of proportional action increases with increasing gain)
        Ti: integral time. (strength of integral action increases with decreasing integral time) 
        Tt: tracking time (has to be smaller than Ti)
        b: weight on the reference speed (value should be set to a number between 0 and 1, usually it is set to 0.5)
        u_min: lower bound of the controller output 
        u_max: upper bound of the controller output 
        """
        
        self.k = k
        self.Ti = Ti
        self.Tt = Tt
        self.b = b
        self.u_min = u_min
        self.u_max = u_max
        
        self.reset();
        
        
    def reset(self):
        
        """ 
        reset the state of the controller
        """ 
        self._p_term = 0.0
        self._i_term = 0.0
        self._last_time = None                   # used for automatic calculation of dt 
        
    def compute_PID_output(self,ref,measured,dt = None):
        
        """
        ref: reference speed
        
        measured: measured speed 
        
        dt: sampling period. 
        
        Note: If dt is set to None, then the system clock will be used to calculate the time eplased since last update 
        
        """
        if dt is None:
            now = time.time()
            # alternatively, you can use ROS's time module to get the current time (This is the preferred method)
            # now = rospy.get_time() 
            if self._last_time is None:
                self._last_time = now
                
            dt = now - self._last_time
            self._last_time = now
        
        self._p_term = self.k * (self.b * ref - measured)
        
        u_temp = self._p_term + self._i_term
        
        # limit the controller output 
        if u_temp > self.u_max:
            u = self.u_max
        elif u_temp < self.u_min:
            u = self.u_min
        else:
            u = u_temp
                
        # update the integral term        
        self._i_term = self._i_term + (self.k * dt / self.Ti) * (ref - measured) +  (dt / self.Tt ) * (u - u_temp);
        
        # return the controller output 
        return u

class Stanley(object):
    
    def __init__(self):
        
    # receive vehicle information from sensors 
        
    def GNSS_callback(self):
        
    def vehicleSpeed_callback(self):
        
    def steeringAngle_callback(self): 
        
    
def PID_test():
    
   # Initialize controller parameters
  refSpeed = 0.0
  measuredSpeed = 0.0
        
  pid_object = PID(1.0,100,10,0.5,-1000,1000)
  print("pid test")
        
  while True:
              
    output = pid_object.compute_PID_output(refSpeed,measuredSpeed)
    print(output)
         

if __name__ == '__main__':
    
    PID_test()
        
            
            

    
