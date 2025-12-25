#!/usr/bin/env python
# coding: utf-8

# In[3]:


import numpy as np

# Lateral control

# TODO: write the PID controller using what you've learned in the previous activities

# Note: y_hat will be calculated based on your DeltaPhi() and poseEstimate() functions written previously 

def PIDController(
    v_0, # assume given (by the scenario)
    y_ref, # assume given (by the scenario)
    y_hat, # assume given (by the odometry)
    prev_e_y, # assume given (by the previous iteration of this function)
    prev_int_y, # assume given (by the previous iteration of this function)
    delta_t): # assume given (by the simulator)
    """
    Args:
        v_0 (:double:) linear Duckiebot speed.
        y_ref (:double:) reference lateral pose
        y_hat (:double:) the current estiamted pose along y.
        prev_e_y (:double:) tracking error at previous iteration.
        prev_int_y (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        v_0 (:double:) linear velocity of the Duckiebot 
        omega (:double:) angular velocity of the Duckiebot
        e_y (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        e_int_y (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """

    # TODO: these are random values, you have to implement your own PID controller in here
    # PD resonant = P 12 D 2.65
    # Original odometry  
    # k_p = 15.5
    # k_i = 0.0
    # k_d = 220.0
    k_p = 3.5
    k_i = 0.0
    k_d = 75.0

    i_max = 100.0 
    
    e_y = y_ref - y_hat
    
    # Initial values in case of reset
    e_int_y = 0.0
    e_der = 0.0
    omega = 0.0
    
    # Reset algorithm
    if delta_t > 2.0 or delta_t < -2.0:
        print(f"PID RESET: delta_t {delta_t:.2f} y_ref {y_ref:.2f} y_hat {y_hat:.2f}  e_y {e_y:.2f} omega: {omega:.2f} e_int_y: {e_int_y:.2f} e_der_y: {e_der:.2f} prev_e: {e_y:.2f} kp {k_p} kd {k_d} ki {k_i} imax {i_max}")
        return [v_0, omega], e_y, e_int_y
    
    e_der = (e_y - prev_e_y)/delta_t
    e_int_y = prev_int_y + e_y * delta_t
    e_int_y = max(min(e_int_y,i_max),-i_max)
    # if abs(e_y) < 0.05:
    #    e_int_y = 0
        
    omega = k_p * e_y + k_d * e_der + k_i * e_int_y

    omega_max = 5.0
    if np.abs(omega) > omega_max:
        # Anti wind-up
        omega_orig = omega
        omega = np.sign(omega) * omega_max
        print(f"PID omega damp {omega_orig:.2f}")
    #omega_saturation_correction = omega_orig - omega
    #e_int_y -= (1/k_i) * omega_saturation_correction 
    #e_int_y = max(min(e_int_y,4.0),-4.0)
    #print(f"PID omega : {np.rad2deg(omega):.2f} deg  theta_ref {np.rad2deg(theta_ref):.2f} theta_hat {np.rad2deg(theta_hat):.2f} pid_y {pid_y:.2f}")        
    print(f"PID home: delta_t {delta_t:.2f} y_ref {y_ref:.2f} y_hat {y_hat:.2f}  e_y {e_y:.2f} omega: {omega:.2f} e_int_y: {e_int_y:.2f} e_der_y: {e_der:.2f} prev_e: {e_y:.2f} kp {k_p} kd {k_d} ki {k_i} imax {i_max} om {omega_max} ")

    return [v_0, omega], e_y, e_int_y

