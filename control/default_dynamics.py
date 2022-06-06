#/usr/bin/env python3

import numpy as np


class dynamics():
    def __init__(self, motion_generator_function):
        
        # Prepaked dynamics
        time_set, x_pos, y_pos, a_set = motion_generator_function()
        
        self.time_set = time_set
        self.xpos_set = x_pos
        self.ypos_set = y_pos
        self.attitude_set = a_set
        
        # TODO: adapt function for relatice control (aka state_(n) --> state_(n+1))
    
    def get_plot_data(self, fig, ax):
        
        # TODO: Precompute the behaviour (ideal)
        # TODO: store resulting arrays
        
        # plot results nicely

        
    






def circle_motion(T=8.0, R=0.20, dt=0.1):
    
    time_set = np.arange(0,T,dt)
    x_pos = R*np.cos(time_set * 2*np.pi/T)
    y_pos = R*np.sin(time_set * 2*np.pi/T)
    a_set = np.arange(0, 2*np.pi, 2*np.pi/len(time_set))
    
    return time_set, x_pos, y_pos, a_set
    

def relative_motion_demo():
    raise NotImplementedError
 