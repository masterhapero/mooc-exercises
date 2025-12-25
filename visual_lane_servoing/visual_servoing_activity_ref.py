#!/usr/bin/env python
# coding: utf-8

# In[4961]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np
import math


def get_steer_matrix_left_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_left_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                    using the masked left lane markings (numpy.ndarray)
    """

    h=shape[0]
    w=shape[1]
    hup = math.ceil(h*2/7)
    halfw = math.ceil(w/2)
    # res = np.ones(shape=shape, dtype="float32")
    # res [0, 0] = 0
    res = np.zeros(shape=shape, dtype="float32")
    mul = 1
    base = 0
    # 0.15 speed = excellent
    # bfactor = 1.0
    # lspoff2 = 35
    # lsblur = 7.5 
    # lstart = 70
    # lmax = 15.5
    # tfac = 1.0
    # lspoff = 13
    # lspdiv = 460
    # woff = 18  
    # bw = 8
    bfactor = 0.3 / 0.22
    lsblur = 7.5 
    lstart = 68
    lmax = 15.5
    # tfac = 1.0
    # 2021_06_03_00_00_57 was good with right side push
    lspoff = 21
    lspdiv = 460
    lwoff = 0
    bw = 32
    xfac = 2.95
    lspoff2 = 0
    # lwoff = 18
    # bw = 10
    # xfac = 5.0
    # lwoff = 20
    # bw = 11
    # xfac = 6.2
    lfill = 0.0
    speed = 0.3
    speedfactor = 1.0

    print(f'get_steer_matrix_left_lane_markings: lo {lspoff} lsd {lspdiv} tf {bfactor} ls {lsblur} lstart {lstart}')
    dir = -1
    # Logic for getting mid track
    for x in range(lstart,halfw):
        cur = base+mul*x
        tfactor = bfactor
        sp_ = lspoff + int(math.ceil(np.square(halfw-x)/lspdiv))            
        res[max(sp_,0):h, cur:(cur+1)] = 0.15*dir*tfactor
        sp_ = sp_ + int(5*speedfactor)
        res[max(sp_,0):h, cur:(cur+1)] = 0.25*dir*tfactor
        sp_ = sp_ + int(5*speedfactor)
        res[max(sp_,0):h, cur:(cur+1)] = 0.30*dir*tfactor
        sp_ = sp_ + int(5*speedfactor)
        #res[max(sp_,0):h, cur:(cur+1)] = dir*min(0.25*tfactor,lmax)
        #sp_ = sp_ + 10
        res[max(sp_,0):h, cur:(cur+1)] = 0.35* dir*tfactor
        sp_ = sp_ + int(5*speedfactor)
        #if speed > 0.25:
        #    if x < 100:
        #        sp_ = sp_ + 50
        res[max(sp_,0):h, cur:(cur+1)] = dir*min(lfill*tfactor,lmax)

    if speed < 0.25:
        lsp_ = 0
        xa = halfw - 20
        lxl = int(12*speedfactor)
        ls = 50
        lxplus = 2.0
        for x in range(xa-lxl,xa+lxl):
            tfactor = bfactor
            cur = base+mul*x
            sp_ = lsp_
            #+int(math.ceil(np.square(abs(xa-x))))                                                                                    
            res[max(sp_,0):max(sp_,0)+10, cur:(cur+1)] = dir * lxplus * tfactor # rxplus
            # res[max(sp_,0):max(sp_,0)+10, cur:(cur+1)] = dir * xfac * tfactor # rxplus
    else: 
        xa = halfw - 35
        lxl = 6
        lxplus = 1.0
        for x in range(xa-lxl,xa+lxl):
            tfactor = bfactor
            cur = base+mul*x
            #sp_ = int(lspoff/12) + int(math.ceil(np.square(halfw-x)/lspdiv))
            #res[max(sp_,0):max(sp_,0)+20, cur:(cur+1)] = dir * lxplus * tfactor # rxplus
            res[0:10, cur:(cur+1)] = dir * lxplus * tfactor # rxplus
#        xa = halfw - 70
#        lxl = 8
#        lxplus = 0.5
#        for x in range(xa-lxl,xa+lxl):
#            tfactor = bfactor
#            cur = base+mul*x
#            sp_ = 5
#            res[max(sp_,0):max(sp_,0)+10, cur:(cur+1)] = dir * lxplus * tfactor # rxplus

        
    # Logic for turning
    # for x in range(235+woff,halfw+5+woff):
    # for x in range(240+lwoff,halfw+bw+lwoff):
    #    cur = base+mul*x
    #    tfactor = bfactor
    #    res[0:h, cur:(cur+1)] = dir*0.8
    #    tfactor = bfactor * xfac * (x-lwoff-220) / (halfw+bw-240)
    #    tfactor2 = bfactor * xfac * (x-lwoff-227) / (halfw+bw-240)
    #    tfactor3 = bfactor * xfac * (x-lwoff-234) / (halfw+bw-240)
    #    sp_ = lspoff2 # + int(math.ceil(np.square(halfw+lwoff+bw-x)/lspdiv))
    #    if speed < 0.25:
    #        res[max(sp_,0):h, cur:(cur+1)] = dir*tfactor
    #        sp_ = sp_ + 15
    #        res[max(sp_,0):h, cur:(cur+1)] = dir*tfactor2
    #        sp_ = sp_ + 15
    #    res[max(sp_,0):h, cur:(cur+1)] = dir*tfactor3

    lwoff = 29
    bw = 60
    # tfac2 = 3.15
    # xfac2 = 3.25
    # xfac2 = 3.37 too little / 3.9 too much
    xfac2 = 3.53
    lstartx = 235
    if speed > 0.25:
        lxstart = 205 # 217 was quite good
    # print(f'LOOP START {lxstart+lwoff} LOOP END {halfw+bw+lwoff}')
    xfac2 = xfac2/(51+(halfw+bw-lxstart)/2)*51
    for x in range(lxstart+lwoff,halfw+bw+lwoff):
        cur = base+mul*x
        tfactor = bfactor
        sp_ = lspoff2 + int(math.ceil(np.square(halfw-x)/lspdiv))
        if speed < 0.25:
            if x > 235+lwoff:
                tfactor = tfactor * 1.0
            if x > 247+lwoff:
                tfactor = tfactor * 1.5
            if x > 249+lwoff:
                tfactor = tfactor * xfac2
                res[max(sp_,0):h, cur:(cur+1)] = dir*min(0.55*tfactor,lmax)
                sp_ = sp_ + 10
                res[max(sp_,0):h, cur:(cur+1)] = dir*min(0.6*tfactor,lmax)
                sp_ = sp_ + 10
                res[max(sp_,0):h, cur:(cur+1)] = dir*min(0.7*tfactor,lmax)
                sp_ = sp_ + 10
                res[sp_:h, cur:(cur+1)] = dir*min(0.7*tfactor,lmax)
                sp_ = sp_ + 10
                res[sp_:h, cur:(cur+1)] = dir*min(0.9*tfactor,lmax)
                sp_ = sp_ + 10
                res[sp_:h, cur:(cur+1)] = dir*min(1.0*tfactor,lmax)
                sp_ = sp_ + 10
                res[sp_:h, cur:(cur+1)] = dir*min(1.0*tfactor,lmax)
        else:
            tfactor = tfactor * xfac2
            res[0:h, cur:(cur+1)] = dir*0.5*tfactor 
            if x > 230:
                res[0:h, cur:(cur+1)] = dir*0.65*tfactor 
            if x > 240:
                res[0:h, cur:(cur+1)] = dir*0.65*tfactor 
            if x > 260:
                res[0:h, cur:(cur+1)] = dir*0.7*tfactor 
            res[0:5, cur:(cur+1)] = dir*0.7*tfactor
            res[30:h, cur:(cur+1)] = dir*0.95*tfactor 
            res[50:h, cur:(cur+1)] = dir*1.36*tfactor 
            # Might need PD revisiong
            # res[sp_+20:h, cur:(cur+1)] = dir*1.36*tfactor 

        
        # for x in range(xa-lxl,xa+lxl):
        #    cur = base+mul*x
        #    res[0:10, cur:(cur+1)] = dir * xfac * bfactor
    res = cv2.GaussianBlur(res,(0,0), lsblur)
    res[0:h, halfw+bw+lwoff:w] = 0.0
    # res[:,int(np.floor(w/5*3)):w + 1] = 0
    return res


# In[4962]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

def get_steer_matrix_right_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_right_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                     using the masked right lane markings (numpy.ndarray)
    """

    #res = np.ones(shape=shape, dtype="float32")
    #res [0, 0] = 0
    h=shape[0]
    w=shape[1]
    hup = math.ceil(h*2/7)
    halfw = math.ceil(w/2)
    res = np.zeros(shape=shape, dtype="float32")
    base = w
    mul = -1
    dir = 1
    bfactor = 0.3 / 0.22
    rspoff = -32
    rspoff2 = 0
    rspdiv = 330
    rsblur = 7.5
    rstart = 47
    rmax = 15.5
    woff = 13
    mfactor = 4.75
    lfill = 0.0
    rlowf = 0.25
    speed = 0.3
    speedfactor = 1.0
    
    print(f'get_steer_matrix_right_lane_markings: rspoff {rspoff} rspdiv {rspdiv} tfactor {bfactor}')
    for x in range(rstart,halfw):
        cur = base+mul*x
        tfactor = bfactor        
        sp_ = rspoff + int(math.ceil(np.square(halfw-x)/rspdiv))            
        res[max(sp_,0):h, cur:(cur+1)] = 0.07*dir*tfactor
        sp_ = sp_ + int(6*speedfactor)
        res[max(sp_,0):h, cur:(cur+1)] = 0.15*dir*tfactor
        sp_ = sp_ + int(6*speedfactor)
        res[max(sp_,0):h, cur:(cur+1)] = 0.20*dir*tfactor 
        sp_ = sp_ + int(6*speedfactor)
        res[max(sp_,0):h, cur:(cur+1)] = dir*min(0.25*tfactor,rmax)
        sp_ = sp_ + int(8*speedfactor)
        res[max(sp_,0):h, cur:(cur+1)] = dir*min(0.3*tfactor,rmax)
        sp_ = sp_ + int(4*speedfactor)
        res[max(sp_,0):h, cur:(cur+1)] = dir*min(0.15*tfactor,rmax)
        sp_ = sp_ - 10
        res[max(sp_,0):h, cur:(cur+1)] = dir*min(lfill*tfactor,rmax)

    res = cv2.GaussianBlur(res,(0,0), rsblur)
    for x in range(180,halfw):
        cur = base+mul*x
        tfactor = bfactor * mfactor
        sp_ = rspoff + int(math.ceil(np.square(halfw-x)/rspdiv))            
        res[max(sp_,0):h, cur:(cur+1)] = 0.07*dir*tfactor
        sp_ = sp_ + int(6*speedfactor)
        res[max(sp_,0):h, cur:(cur+1)] = 0.15*dir*tfactor
        sp_ = sp_ + int(6*speedfactor)
        res[max(sp_,0):h, cur:(cur+1)] = 0.20*dir*tfactor 
        sp_ = sp_ + int(6*speedfactor)
        res[max(sp_,0):h, cur:(cur+1)] = dir*min(0.25*tfactor,rmax)
        sp_ = sp_ + int(8*speedfactor)
        res[max(sp_,0):h, cur:(cur+1)] = dir*min(0.3*tfactor,rmax)
        sp_ = sp_ + int(4*speedfactor)
        sp_ = rspoff2     
        res[sp_:h, cur:(cur+1)] = dir*0.350*tfactor
        res[sp_+10:h, cur:(cur+1)] = dir*0.390*tfactor
        res[sp_+15:h, cur:(cur+1)] = dir*0.410*tfactor
        res[sp_+20:h, cur:(cur+1)] = dir*0.425*tfactor
        res[sp_+25:h, cur:(cur+1)] = dir*0.430*tfactor
        res[sp_+30:h, cur:(cur+1)] = dir*0.350*tfactor
        res[sp_+35:h, cur:(cur+1)] = dir*0.350*tfactor
        res[sp_+45:h, cur:(cur+1)] = dir*0.350*tfactor
            
    rsp_ = 0
    xa = halfw - 102
    rxl = 8
    rxplus = 1.0
    for x in range(xa-rxl,xa+rxl):
        tfactor = bfactor
        cur = base+mul*x
        # sp_ = rsp_
        sp_ = rspoff + int(math.ceil(np.square(halfw-x)/rspdiv))
        #+int(math.ceil(np.square(abs(xa-x))))                                                                                    
        res[max(sp_,0):max(sp_,0)+20, cur:(cur+1)] = dir * rxplus * tfactor # rxplus

    rsp_ = 0
    xa = halfw - 80
    rxl = 7
    xs = 50
    rxplus = 2.90
    for x in range(xa-rxl,xa+rxl):
        tfactor = bfactor
        cur = base+mul*x
        sp_ = rsp_
        #+int(math.ceil(np.square(abs(xa-x))))                                                                                    
        res[max(sp_,0):max(sp_,0)+int(13*speedfactor), cur:(cur+1)] = dir * rxplus * tfactor # rxplus

    # Kick back to lane
    # rkickback = 0.23
    if speed < 0.25:
        rkickback = 2.2 * speedfactor
        for x in range(0,25):
            cur = base+mul*x
            # tp = 40-x
            tp = int(x / 2)
            # rkick = 0.3+(50-x/2)/100
            #res[0:tp, cur:(cur+1)] = dir * rkickback * tfactor # rxplus
            res[tp:tp+int(4*speedfactor), cur:(cur+1)] = dir * rkickback * tfactor
            
    # rsp_ = 0
    # xa = halfw - 95
    # rxl = int(3*speedfactor)
    # rxplus = 0.7
    #for x in range(xa-rxl,xa+rxl):
    #    tfactor = bfactor
    #    cur = base+mul*x
    #    sp_ = rsp_
    #    res[max(sp_,0):max(sp_,0)+int(10*speedfactor), cur:(cur+1)] = dir * rxplus * tfactor # rxplus
    res = cv2.GaussianBlur(res,(0,0), rsblur)

    rsp_ = 0
    xa = halfw - 105
    rxl = 14
    rxplus = 0.95
    for x in range(xa-rxl,xa+rxl):
        tfactor = bfactor
        cur = base+mul*x
        sp_ = rsp_
        res[max(sp_,0):max(sp_,0)+5, cur:(cur+1)] = dir * rxplus * tfactor # rxplus

    # res *= 
    
    res[:,0:int(np.floor(w/5*3))-10] = 0
    return res


# In[4963]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def detect_lane_markings(image):
    """
        Args:
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
            right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
    """
    
    h, w, _ = image.shape
    
    threshold = 70.0
    sigma = 2.5

    # Convert the image to HSV for any color-based filtering
    imagehsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Most of our operations will be performed on the grayscale version
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    img_gaussian_filter = cv2.GaussianBlur(img,(0,0), sigma)
    
    # mask_ground = np.ones(img.shape, dtype=np.uint8) # TODO: CHANGE ME
    # mask_ground[0:170, 0:w] = 0.0
    white_lower_hsv = np.array([0, 0, 150])   
    white_upper_hsv = np.array([179, 40, 255])  
    yellow_lower_hsv = np.array([25, 70, 70])       
    yellow_upper_hsv = np.array([30, 255, 255])  

    mask_white = cv2.inRange(imagehsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imagehsv, yellow_lower_hsv, yellow_upper_hsv)
    
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)

    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)

    # Compute the orientation of the gradients
    Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, dtype=np.float32), angleInDegrees=True)
    
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)
    mask_mag = (Gmag > threshold)

    width = img.shape[1]
    mask_left = np.ones(sobelx.shape)
    # narrow
    # mask_left[:,int(np.floor(width/5*2)):width + 1] = 0
    # wide
    mask_left[:,int(np.floor(width/5*3))+20:width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    # wide
    mask_right[:,0:int(np.floor(width/5*1))] = 0
    # narrow
    # mask_right[:,0:int(np.floor(width/5*3))] = 0

    
    mask_left_edge = mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white
    mask_left_edgefilt = (mask_left_edge > 240)
    mask_right_edgefilt = (mask_right_edge > 240)
        
    #mask_left_edge = np.random.rand(h, w)
    #mask_right_edge = np.random.rand(h, w)
    
    return (mask_left_edge*mask_left_edgefilt, mask_right_edge*mask_right_edgefilt)

