from typing import Tuple

import numpy as np
import math

def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape=shape, dtype="float32")  # write your function instead of this one
    base = 640
    mul = -1
    for x in range(80,320):
        cur = base+mul*x
        sp_ = 165+int(math.ceil(np.square(320-x)/180))
        res[sp_:480, cur:(cur+1)] = -0.4+0.35*x/300
        #(x*x)/(360*360)
        sp_ += 10
        res[sp_:480, cur:(cur+1)] = -0.0
        sp_ += 50
        res[sp_:480, cur:(cur+1)] = -0.3
        sp_ += 20
        res[sp_:480, cur:(cur+1)] = -0.4
        sp_ += 35
        res[sp_:480, cur:(cur+1)] = -0.4
        sp_ += 35
        res[sp_:480, cur:(cur+1)] = -0.5
        sp_ += 35
        res[sp_:480, cur:(cur+1)] = -0.5

        #sp_ = 195+int(math.ceil(np.square(320-x)/240))
        # sp_ -= 45
        #if x > 120:
        #    res[sp_:480, cur:(cur+1)] = -0.025
        #sp_ += 20
        #if x > 100:
        #    res[sp_:480, cur:(cur+1)] = -0.05
        #sp_ += 50
        #if x > 90:
        #    res[sp_:480, cur:(cur+1)] = -0.2
        #sp_ += 35
        #if x > 85:
        #    res[sp_:480, cur:(cur+1)] = -0.7
        #sp_ += 35
        #if x > 80:
        #    res[sp_:480, cur:(cur+1)] = -0.9
        #sp_ += 35
        #res[sp_:480, cur:(cur+1)] = -1.23
        # res[sp_:480, cur:(cur+1)] = -np.exp2(np.linspace(0.8,mv,480-sp_)).reshape(480-sp_,1)/16

    #kernel = np.array([0.025,0.05,0.1,0.25,0.5,0.75,0.5,0.25,0.1,0.05,0.025])
    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, res)
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 1
    #b /= np.abs(np.min(b))
    
    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b) # 
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 2
    #b /= np.abs(np.min(b))

    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b) # 
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 3
    #b /= np.abs(np.min(b))
    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b) #
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 4
    #b /= np.abs(np.min(b))
    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b) #
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 5
    #b /= np.abs(np.min(b))
    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b) # 
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 6
    #b /= np.abs(np.min(b))
    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b) # 
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 7
    #b /= np.abs(np.min(b))

    #b[0:480, 0:320]=0.0
    return res

def get_motor_left_matrix_gauss(shape: Tuple[int, int]) -> np.ndarray:
    res = get_motor_left_matrix(shape)
    kernel = np.array([0.1,0.25,0.5,1.0,2.0,4.0,2.0,1.0,0.5,0.25,0.1])
    a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, res)
    b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a)
    a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b)
    b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a)
    a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b)
    b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a)
    return b
    
def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape=shape, dtype="float32")  # write your function instead of this one

    base = 0
    mul = 1

    for x in range(80,320):
        cur = base+mul*x
        sp_ = 165+int(math.ceil(np.square(320-x)/180))
        res[sp_:480, cur:(cur+1)] = -0.4+0.35*x/300
        # res[sp_:480, cur:(cur+1)] = -(x*x)/(360*360)
        sp_ += 10
        res[sp_:480, cur:(cur+1)] = -0.0
        sp_ += 50
        res[sp_:480, cur:(cur+1)] = -0.3
        sp_ += 20
        res[sp_:480, cur:(cur+1)] = -0.4
        sp_ += 35
        res[sp_:480, cur:(cur+1)] = -0.4
        sp_ += 35
        res[sp_:480, cur:(cur+1)] = -0.5
        sp_ += 35
        res[sp_:480, cur:(cur+1)] = -0.5
        # res[sp_:480, cur:(cur+1)] = -np.exp2(np.linspace(0.8,mv,480-sp_)).reshape(480-sp_,1)/16

    #kernel = np.array([0.025,0.05,0.1,0.25,0.5,0.75,0.5,0.25,0.1,0.05,0.025])
    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, res)
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 1
    #b /= np.abs(np.min(b))
    
    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b) # 
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 2
    #b /= np.abs(np.min(b))

    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b) # 
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 3
    #b /= np.abs(np.min(b))
    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b) #
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 4
    #b /= np.abs(np.min(b))
    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b) #
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 5
    #b /= np.abs(np.min(b))
    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b) # 
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 6
    #b /= np.abs(np.min(b))
    #a = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 0, b) # 
    #b = np.apply_along_axis(lambda x: np.convolve(x, kernel, mode='same'), 1, a) # 7
    #b /= np.abs(np.min(b))

    #b[0:480, 320:640]=0.0
    return res

def get_motor_left_attract_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape=shape, dtype="float32")  # write your function instead of this one
    res[0:220, 0:320] = -1
    res[0:220, 321:640] = 1
    # Smaller ducks in horizon
    res[0:160, 0:320] = -3
    res[0:160, 321:640] = 3
    res[160:190, 0:320] = -2
    res[160:190, 321:640] = 2
    return res
    
def get_motor_right_attract_matrix(shape: Tuple[int, int]) -> np.ndarray:
    return get_motor_left_attract_matrix(shape)

