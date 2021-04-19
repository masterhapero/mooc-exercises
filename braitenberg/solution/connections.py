from typing import Tuple

import numpy as np


def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape=shape, dtype="float32")  # write your function instead of this one
#    res[0:250, 320:639] = 0.01

#    res[260:480, 0:100] = -2
#    res[250:480, 100:200] = -2
#    res[230:480, 200:320] = -4
#    res[200:480, 280:320] = -4
#    res[360:480, 0:320] = -4


    res[200:480, 0:100] = 5
    res[170:480, 100:200] = 10
    res[150:480, 200:320] = 20
    res[150:480, 320:439] = -20
    res[170:480, 440:540] = -10
    res[200:480, 540:639] = -5

    
#    res[260:480, 540:639] = -1
#    res[250:480, 440:539] = -1
#    res[230:480, 320:440] = -1
#    res[360:480, 320:640] = -1
#    res[186:480, 320:360] = -1
    
    # res[0:200, 0:639] = 0
    return res


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape=shape, dtype="float32")  # write your function instead of this one

#    res[0:250, 0:320] = 0.01

#    res[260:480, 0:100] = -1
#    res[250:480, 100:200] = -1
#    res[230:480, 200:320] = -1
#    res[360:480, 0:320] = -1
#    res[170:480, 280:320] = -1

#    res[260:480, 540:639] = -2
#    res[250:480, 440:540] = -2
#    res[230:480, 320:440] = -4
#    res[360:480, 320:640] = -4
#    res[200:480, 320:360] = -4

    #res[0:200, 0:639] = 1
    res[200:480, 0:100] = -5
    res[170:480, 100:200] = -10
    res[150:480, 200:320] = -20
    res[150:480, 320:439] = 20
    res[170:480, 440:540] = 10
    res[200:480, 540:639] = 5

    # res[0:200, 0:639] = 0
    return res
