#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Optional, Tuple

import math
import numpy as np
from aido_schemas import (
    Context,
    DB20Commands,
    DB20Observations,
    EpisodeStart,
    GetCommands,
    JPGImage,
    LEDSCommands,
    protocol_agent_DB20,
    PWMCommands,
    RGB,
    wrap_direct,
)

import duckietown_code_utils as dcu
from connections import get_motor_left_matrix, get_motor_right_matrix, get_motor_left_attract_matrix, get_motor_right_attract_matrix
from preprocessing import preprocess


@dataclass
class BraitenbergAgentConfig:
#   14482: gain: float = 0.20
    gain: float = 0.20
    gainFree: float = 0.3
    const: float = 0.3
    constFree: float = 0.2


class BraitenbergAgent:
    config = BraitenbergAgentConfig()

    left: Optional[np.ndarray]
    right: Optional[np.ndarray]
    rgb: Optional[np.ndarray]
    l_max: float
    r_max: float
    l_min: float
    r_min: float
    x_min: float
    x_min: float

    
    def init(self, context: Context):
        context.info("init()")
        self.rgb = None
        self.l_max = -math.inf
        self.r_max = -math.inf
        self.avoid_max = -math.inf
        self.attract_max = -math.inf
        self.l_min = math.inf
        self.r_min = math.inf
        self.avoid_min = math.inf
        #self.avoid_min = -10000.0
        # 22 was good for parabolic (not boost pixel)
        # 275 was max tested
        self.avoid_init = -275.0
        self.avoid_min = self.avoid_init
        self.attract_min = math.inf
        self.left = None
        self.right = None
        self.left_attract = None
        self.right_attract = None
        print(f'Quack quack! Entering arena with avoid_init {self.avoid_init} const {self.config.const} gain {self.config.gain}.')
        
    def on_received_seed(self, data: int):
        np.random.seed(data)

    def on_received_episode_start(self, context: Context, data: EpisodeStart):
        context.info(f'Starting episode "{data.episode_name}".')

    def on_received_observations(self, context: Context, data: DB20Observations):
        camera: JPGImage = data.camera
        if self.rgb is None:
            context.info("received first observations")
        self.rgb = dcu.bgr_from_rgb(dcu.bgr_from_jpg(camera.jpg_data))

    def compute_commands(self) -> Tuple[float, float]:
        """ Returns the commands (pwm_left, pwm_right) """
        # If we have not received any image, we don't move
        if self.rgb is None:
            return 0.0, 0.0

        if self.left is None:
            # if it is the first time, we initialize the structures
            shape = self.rgb.shape[0], self.rgb.shape[1]
            self.left = get_motor_left_matrix(shape)
            self.right = get_motor_right_matrix(shape)
            self.left_attract = get_motor_left_attract_matrix(shape)
            self.right_attract = get_motor_right_attract_matrix(shape)

        # let's take only the intensity of RGB
        P = preprocess(self.rgb)
        # now we just compute the activation of our sensors
        l = float(np.sum(P * self.left))
        r = float(np.sum(P * self.right))

        la = float(np.sum(P * self.left_attract))
        ra = float(np.sum(P * self.right_attract))

        # These are big numbers -- we want to normalize them.
        # We normalize them using the history

        # first, we remember the high/low of these raw signals
        self.l_max = max(l, self.l_max)
        self.r_max = max(r, self.r_max)
        self.l_min = min(l, self.l_min)
        self.r_min = min(r, self.r_min)

        self.avoid_max = max(l, self.avoid_max)
        self.avoid_max = max(r, self.avoid_max)
        self.avoid_min = min(l, self.avoid_min)
        self.avoid_min = min(r, self.avoid_min)

        self.attract_max = max(la, self.attract_max)
        self.attract_max = max(ra, self.attract_max)
        self.attract_min = min(la, self.attract_min)
        self.attract_min = min(ra, self.attract_min)

        # now rescale from 0 to 1
        #ls = rescale(l, self.l_min, self.l_max)
        #rs = rescale(r, self.r_min, self.r_max)

        ls = rescale(l, self.avoid_min, self.avoid_max)
        rs = rescale(r, self.avoid_min, self.avoid_max)

        # down scale signal factors to reasonable

        if self.avoid_min < 1.15*self.avoid_init and np.abs(ls) > 0.8 and np.abs(rs) > 0.8:
            print(f'Downscaling avoid_min "{self.avoid_min}" ls "{ls:.2f}" rs "{rs:.2f}" .')
            self.avoid_min /= 1.1
            # if self.avoid_min < -500.0:
            #    print(f'Downscaling (2) avoid_min "{self.avoid_min}".')
            #    self.avoid_min /= 10.0
            #if self.avoid_min < -500.0:
            #    print(f'Downscaling (3) avoid_min "{self.avoid_min}".')
            #    self.avoid_min /= 10.0
        
        las = rescale(la, self.attract_min, self.attract_max)
        ras = rescale(ra, self.attract_min, self.attract_max)
        
        gain = self.config.gain
        gainFree = self.config.gainFree
        const = self.config.const
        constFree = self.config.constFree

        pwm_left = const + ls * gain
        pwm_right = const + rs * gain

        print(f'Signals avoid_min "{self.avoid_min}" l "{l:.2f}" r "{r:.2f}" ls "{ls:.2f}" rs "{rs:.2f}" pwml "{pwm_left:.2f}" pwmr "{pwm_right:.2f}".')
        
        # if l != 0.0 or r != 0.0:
        #    pwm_left = const + ls * gain
        #    pwm_right = const + rs * gain
        #else:
        #    if la != 0.0 or ra != 0.0:
        #        pwm_left = constFree + las * gainFree
        #        pwm_right = constFree + ras * gainFree
        #    else:
        #        pwm_left = const
        #        pwm_right = -const
        return pwm_left, pwm_right

    def on_received_get_commands(self, context: Context, data: GetCommands):
        pwm_left, pwm_right = self.compute_commands()

        col = RGB(0.0, 0.0, 1.0)
        col_left = RGB(pwm_left, pwm_left, 0.0)
        col_right = RGB(pwm_right, pwm_right, 0.0)
        led_commands = LEDSCommands(col, col_left, col_right, col_left, col_right)
        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        commands = DB20Commands(pwm_commands, led_commands)
        context.write("commands", commands)

    def finish(self, context: Context):
        context.info(f'End x min "{self.avoid_min}" max "{self.avoid_max}".')
        context.info("finish()")


def rescale(a: float, L: float, U: float):
    if np.allclose(L, U):
        return 0.0
    return (a - L) / (U - L)


def main():
    node = BraitenbergAgent()
    protocol = protocol_agent_DB20
    wrap_direct(node=node, protocol=protocol)


if __name__ == "__main__":
    main()
