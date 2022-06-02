#/usr/bin/env python3

import numpy as np


def circle_motion(T=8.0, R=0.20, dt=0.1):
    
    time_set = np.arange(0,T,dt)
    x_pos = R*np.cos(time_set * 2*np.pi/T)
    y_pos = R*np.sin(time_set * 2*np.pi/T)
    a_set = np.arange(0, 2*np.pi, 2*np.pi/len(time_set))
    
    return time_set, x_pos, y_pos, a_set
    

def relative_motion_demo():
    raise NotImplementedError
 