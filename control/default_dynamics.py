#/usr/bin/env python3

import numpy as np
import time
import matplotlib.pyplot as plt



def swarm_relative_test(platforms, idx, running_time):   
    # properties
    hold_position = [0.0, 0.0]
    period = 20
    
    r = 0.9
    
    if idx == 0: # perform absolute motion
        x,y = hold_position
        a = ((running_time/period)%1) * 2*np.pi
        
        return x,y,a
    else:
        primairy_platform = platforms[0]
        xt,yt,at = primairy_platform.xpos, primairy_platform.ypos, primairy_platform.attitude
        
        x = r*np.cos(at) + xt
        y = r*np.sin(at) + yt
        a = (-at)%(2*np.pi)
        
        return x,y,a
        
    


def transform_frame(data, transform=[0,0,0]):
    t_data, x_data, y_data, a_data = data
    dx, dy, da = transform
    
    x_transformed = x_data + dx
    y_transformed = y_data + dy
    a_transform = a_data + da # check if behaving
    
    return t_data, x_transformed, y_transformed, a_transform
    
    
def stay_at_position(dt = 0.1):
    time_set = time_set=np.arange(0, 10, dt)
    
    x_pos = np.zeros(len(time_set))
    y_pos = np.zeros(len(time_set))
    a_set = np.zeros(len(time_set))
    
    return [time_set, x_pos, y_pos, a_set], [time_set, x_pos, y_pos, a_set]
    
    



def circle_around_point(point_pos, point_vel, time, radius, period, sign=1, initial_time=0, initial_angle=0):
    center_px, center_py = point_pos
    center_vx, center_vy = point_vel
    
    rate = (2*np.pi)/period
    delta_time = (time-initial_time)
    
    # Position calculation
    pos_x = center_px + radius * np.cos(sign*rate*delta_time)
    pos_y = center_py + radius * np.sin(sign*rate*delta_time)
    
    # velocity calculation
    vel_x = center_vx - sign*rate*radius*np.sin(sign*rate * delta_time)
    vel_y = center_vy + sign*rate*radius*np.cos(sign*rate * delta_time)
    
    return pos_x, pos_y, vel_x, vel_y
    

def swarm_circle(f1=1, f2=1, R1=0.4, R2=0.9, P=30, dt=0.1):
    gcd = np.gcd(f1,f2)
    end_time = gcd*P
    
    time_set=np.arange(0, end_time, dt)
    
    pos1x, pos1y, dir1x, dir1y = circle_around_point([0,0], [0,0], time_set, R1, P/f1, sign=-1)
    pos2x, pos2y, dir2x, dir2y = circle_around_point([pos1x, pos1y], [dir1x, dir1y], time_set, R2, P/f2, sign=-1)
    
    relx, rely = pos1x - pos2x, pos1y - pos2y
    dir2_to_1 = (np.arctan2(relx, rely))

    dir_robot_1 = (np.arctan2(-relx, -rely))
    
    set_1 = [time_set, pos1x, pos1y, dir_robot_1]
    set_2 = [time_set, pos2x, pos2y, dir2_to_1]
    
    return set_1, set_2


def calibrate_directions(size = 0.6, seq_time=2, dt=0.1):
    
    seq_dir = [0*np.pi, 0.5*np.pi, 1.0*np.pi, 1.5*np.pi]
    N = len(seq_dir)
    time_set = np.arange(0,2*seq_time*N,dt)
    
    x_pos = np.zeros(len(time_set))
    y_pos = np.zeros(len(time_set))
    a_set = np.zeros(len(time_set))
    
    for i, dir in enumerate(seq_dir):
        seq_start_time = i*2*seq_time
        seq_half_time = i*2*seq_time + seq_time
        seq_end_time = (i+1)*2*seq_time
        
        progress_out  = np.linspace(0,1,int(seq_time/dt))
        progress_back = np.linspace(1,0,int(seq_time/dt))

        x_pos[int(seq_start_time/dt):int(seq_half_time/dt)] = size * np.cos(dir) * smooth_progress(progress_out)
        y_pos[int(seq_start_time/dt):int(seq_half_time/dt)] = size * np.sin(dir) * smooth_progress(progress_out)
        
        x_pos[int(seq_half_time/dt):int(seq_end_time/dt)]   = size * np.cos(dir) * smooth_progress(progress_back)
        y_pos[int(seq_half_time/dt):int(seq_end_time/dt)]   = size * np.sin(dir) * smooth_progress(progress_back)
    
    return [time_set, x_pos, y_pos, a_set], [time_set, x_pos, y_pos, a_set]
    
def smooth_progress(linear_progress):
    return  .5*np.sin(linear_progress*np.pi - np.pi/2) + .5

def circle_motion(T=6.0, R=0.25, dt=0.1):
    
    time_set = np.arange(0,T,dt)
    x_pos = R*np.cos(sign*time_set * 2*np.pi/T)
    y_pos = R*np.sin(sign*time_set * 2*np.pi/T)
    a_set = np.arange(0, 2*np.pi, 2*np.pi/len(time_set))
    
    return time_set, x_pos, y_pos, a_set
    


def main():
    #calibrate_directions()
    
    x = np.linspace(0,1,50)
    y = smooth_progress(x)
    plt.plot(x,y)
    plt.show()
    
    

if __name__ == "__main__":
    main()
 
 
 
 