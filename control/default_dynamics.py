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
    

def swarm_circle(f1=1, f2=3, R1=0.5, R2=1, P=80, dt=0.1):
    gcd = np.gcd(f1,f2)
    end_time = gcd*P
    
    time_set=np.arange(0, end_time, dt)
    
    pos1x, pos1y, dir1x, dir1y = circle_around_point([0,0], [0,0], time_set, R1, P/f1)
    pos2x, pos2y, dir2x, dir2y = circle_around_point([pos1x, pos1y], [dir1x, dir1y], time_set, R2, P/f2)
    
    relx, rely = pos1x - pos2x, pos1y - pos2y
    dir2_to_1 = (np.arctan2(relx, rely) + np.pi)

    dir_robot_1 = (np.arctan2(-relx, -rely) + np.pi)
    
    set_1 = [time_set, pos1x, pos1y, dir_robot_1]
    set_2 = [time_set, pos2x, pos2y, dir2_to_1]
    
    return set_1, set_2


def circle_motion(T=6.0, R=0.25, dt=0.1):
    
    time_set = np.arange(0,T,dt)
    x_pos = R*np.cos(time_set * 2*np.pi/T)
    y_pos = R*np.sin(time_set * 2*np.pi/T)
    a_set = np.arange(0, 2*np.pi, 2*np.pi/len(time_set))
    
    return time_set, x_pos, y_pos, a_set
    



def main():
    swarm_circle()
    

if __name__ == "__main__":
    main()
 
 
 
 