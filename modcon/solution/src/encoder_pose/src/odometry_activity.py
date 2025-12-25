#!/usr/bin/env python
# coding: utf-8

# In[76]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

#TODO: write a correct function

def DeltaPhi(encoder_msg, prev_ticks):
    """
        Args:
            encoder_msg: ROS encoder message (ENUM)
            prev_ticks: Previous tick count from the encoders (int)
        Return:
            rotation_wheel: Rotation of the wheel in radians (double)
            ticks: current number of ticks (int)
    """
    N_tot = encoder_msg.resolution # number of ticks per wheel revolution
    ticks = encoder_msg.data # incremental count of ticks from the encoder
    # TODO: these are random values, you have to implement your own solution in here
    delta_ticks = ticks - prev_ticks 
    alpha = 2*np.pi/N_tot 
    delta_phi = delta_ticks*alpha

    return delta_phi, ticks


# In[77]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your odometry function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

# TODO: write the odometry function

import numpy as np 

def poseEstimation( R, # radius of wheel (assumed identical) - this is fixed in simulation, and will be imported from your saved calibration for the physical robot
                    baseline_wheel2wheel, # distance from wheel to wheel; 2L of the theory
                    x_prev, # previous x estimate - assume given
                    y_prev, # previous y estimate - assume given
                    theta_prev, # previous orientation estimate - assume given
                    delta_phi_left, # left wheel rotation (rad)
                    delta_phi_right): # right wheel rotation (rad)
    
    """
        Calculate the current Duckiebot pose using the dead-reckoning approach.

        Returns x,y,theta current estimates:
            x_curr, y_curr, theta_curr (:double: values)
    """
    
    # TODO: these are random values, you have to implement your own solution in here
    d_left = R*delta_phi_left
    d_right = R*delta_phi_right
    omega = (d_right-d_left)/baseline_wheel2wheel
    # -0.000040
    v = (d_left + d_right) / 2
    dt = v / 0.2
    
    if abs(omega) < 0.0001:
        x_delta = v
        y_delta = 0
    else:
        # omega = omega - 0.00016
        radius = v / omega
        x_delta = radius * np.sin(omega)
        y_delta = radius * (1.0 - np.cos(omega))

    theta_curr = theta_prev + omega
    # x_curr = x_prev + x_delta * np.cos(theta_prev) - y_delta * np.sin(theta_prev)
    # y_curr = y_prev + y_delta * np.cos(theta_prev) + x_delta * np.sin(theta_prev)
    x_curr = x_prev + x_delta * np.cos(theta_prev) - y_delta * np.sin(theta_prev)
    y_curr = y_prev + y_delta * np.cos(theta_prev) + x_delta * np.sin(theta_prev)
    
    # x_curr = x_prev + d_A*np.cos(theta_prev)
    # y_curr = y_prev + d_A*np.sin(theta_prev)
    
    if delta_phi_left < -150.0 and delta_phi_right < -150.0:
        print("ODOMETRY RESET TO ZERO")
        theta_curr = np.deg2rad(0.6)
        x_curr = 0.0
        y_curr = 0.2
        
    # print(f"pose: X {x_curr:.4f} y_curr {y_curr:.4f} theta_curr {theta_curr:.4f} d_left {d_left:.5f} d_right: {d_right:.5f} v: {v:.5f} omega: {omega:.5f} x_delta: {x_delta:.4f} y_delta: {y_delta:.4f}")
    #print("pose X"+str(x_curr)+" Y"+str(y_curr)+" "+str(theta_curr))
    return x_curr, y_curr, theta_curr

