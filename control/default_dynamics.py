#/usr/bin/env python3

import numpy as np
import time


class dynamics():
    def __init__(self, devices):
        
        
        self.devices = devices
        self.N = len(self.devices)
        
        self.positions = np.zeros((2,N))
        
        
        # Time
        current_time = time.time
        self.time_of_last_update = current_time
        
        
    def update(time):
        pass
        
        
        # TODO: adapt function for relatice control (aka state_(n) --> state_(n+1))
    
    def compute(time):
        pass
        
    
    
    def get_plot_data(self, fig, ax):
        pass
        # TODO: Precompute the behaviour (ideal)
        # TODO: store resulting arrays
        
        # plot results nicely

        
    
    


def circle_around_point(point_pos, point_vel, time, radius, period, initial_time=0, initial_angle=0):
    center_px, center_py = point_pos
    center_vx, center_vy = point_vel
    
    rate = (2*np.pi)/period
    delta_time = (time-initial_time)
    
    # Position calculation
    pos_x = center_px + radius * np.cos(rate*delta_time)
    pos_y = center_py + radius * np.sin(rate*delta_time)
    
    # velocity calculation
    vel_x = center_vx - rate*radius*np.sin(rate * delta_time)
    vel_y = center_vy + rate*radius*np.cos(rate * delta_time)
    
    return pos_x, pos_y, vel_x, vel_y
    
    
    
    
    
    


def circle_motion(T=8.0, R=0.20, dt=0.1):
    
    time_set = np.arange(0,T,dt)
    x_pos = R*np.cos(time_set * 2*np.pi/T)
    y_pos = R*np.sin(time_set * 2*np.pi/T)
    a_set = np.arange(0, 2*np.pi, 2*np.pi/len(time_set))
    
    return time_set, x_pos, y_pos, a_set
    



def main():
    pass
    

if __name__ == "__main__":
    main()
 
 
 
 