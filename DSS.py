#/usr/bin/env python3

import numpy as np
import time

import matplotlib; matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from communication.wifi_api import open_serial, stream_to, get_ports_dict
from control.default_dynamics import circle_motion, swarm_circle, calibrate_directions, transform_frame
from communication.motive_api import get_body_package_data, setup_client


class Platform:    
    def __init__(self, name:str, idx, com_port:str,  motive_id:int,  xpos:float=.0, ypos:float=.0, attitude:float=.0, transform_set=[0,0,0], update_freq:float=0.1, debug=False, max_lin_vel=0.4, max_ang_vel=2*np.pi) -> None:
         self.name = name
         
         # default position and orientation
         self.xpos      = xpos
         self.ypos      = ypos
         self.pos = np.array([self.xpos, self.ypos])
         
         
         self.attitude   = attitude
         
         # Motor settings
         self.speed     = 0
         self.direction = 0
         self.rotation  = 0
         
         # Create serial link to device
         self.com_port = com_port
#         self.ser_com = open_serial(self.com_port)
         self.ser_com = None
         self.update_freq = update_freq
         self.time_of_last_update = time.time()
         
         self.max_lin_vel = max_lin_vel # m/s
         self.max_ang_vel = max_ang_vel # rad/s
         
         self.debug = debug
         
         self.temp_loop_idx = 0
         self.robot_idx = idx
         
         
         # Motive API data
         self.motive_id = motive_id
         
         
         self.get_location()
         self.tranform_set = transform_set
         
         if self.debug: 
            print(self.name, "on", self.com_port)

    
    def console_print(self, name, val):
        if self.debug:
            print(name,val)
#            print(f"{self.name}: {name}={val}")
    
    
    def test_solo_motion(self):
        magnitude = 100.0
        dir_range = np.arange(0,2*np.pi, 4*np.pi/180)
        rotation = 0.0

        for direction in dir_range:
            data_to_send = [magnitude, direction, rotation]
            stream_to(data_to_send, self.ser_com, debug=False)
            
            self.console_print("data", data_to_send)
            
            time.sleep(0.1) #FIXME: dont use sleep!
         
        
    def test_command(self, rate=30):
        current_time = time.time()
        delta_time = current_time - self.time_of_last_update
        
        if delta_time >= self.update_freq:
            self.time_of_last_update = current_time
            
            # speed and rotation constant
            self.speed = 100
            self.rotation = 0   
            turn_rate = rate*np.pi/180 #rads/sj
            
            # direction changes gradually
            self.direction += turn_rate*delta_time
            if self.direction >= 2*np.pi: #reset direction once it gets too high
                self.direction = self.direction - 2*np.pi
            
            data_to_send = [self.speed, self.direction, self.rotation]
            stream_to(data_to_send, self.ser_com)
            
            
            self.console_print("data", data_to_send)

    def start_motion(self, dynamics, repeat=False):

        time_set,x_set,y_set,a_set = transform_frame(dynamics(dt=self.update_freq)[self.robot_idx], self.tranform_set)
            
        current_time = time.time()
        delta_time = current_time - self.time_of_last_update
        

        if delta_time >= self.update_freq:
            self.time_of_last_update = current_time
            
            # Update locations
            self.get_location()
        
            dx = x_set[self.temp_loop_idx] - self.xpos
            dy = y_set[self.temp_loop_idx] - self.ypos
            dir_angle = np.arctan2(dx,dy) #FIXME: check of total magnitude of velocity exceeds, then deal with it
            
            da_raw = (a_set[self.temp_loop_idx] - self.attitude)
            if abs(da_raw) > np.pi:
                da = (da_raw - np.sign(da_raw)*2*np.pi)%(2*np.pi)
            else:
                da = da_raw
            
            # set max magniutde of deltas
            dxmax = np.abs(np.sin(dir_angle)) * self.max_lin_vel * delta_time
            dymax = np.abs(np.cos(dir_angle)) * self.max_lin_vel * delta_time
            damax = self.max_ang_vel * delta_time
            
            if np.abs(dx) > dxmax:
                self.console_print("Forward step exceeds max: ",[np.abs(dx), dxmax])
                dx = np.sign(dx) * dxmax
                
            if np.abs(dy) > dymax:
                self.console_print("Sideways step exceeds max: ",[np.abs(dy), dymax])
                dy = np.sign(dy) * dymax
                
            if np.abs(da) > damax:
                self.console_print("Attitude step exceeds max: ",[np.abs(da), damax])
                dy = np.sign(da) * damax
            
            # print if capping is taking place
            dt = delta_time

            required_direction  = (np.arctan2(dx, dy) + np.pi/2 - self.attitude) % (2*np.pi)
            required_speed      = np.sqrt(dx**2 + dy**2)/dt * 1e3
            required_rotation   = da/dt
            
            data_to_send = [required_speed, required_direction, required_rotation]
            
            stream_to(data_to_send, self.ser_com)
            self.console_print("data", data_to_send)
            
            
            # hardcoding succes of motion
           # self.xpos     = x_set[self.temp_loop_idx]
           # self.ypos     = y_set[self.temp_loop_idx]
           # self.attitude = a_set[self.temp_loop_idx]
            
            if self.temp_loop_idx == len(time_set)-1:
                self.temp_loop_idx = 0
            else:
                self.temp_loop_idx += 1
                
    def get_location(self):
        xpos_data, ypos_data, attitude_data = get_body_package_data(self.motive_id)
        print(xpos_data)
        # set object values
        self.ypos     = ypos_data
        self.xpos     = xpos_data
        self.attitude = attitude_data
                
    def initalise_position(self, xpos,ypos,attitude):     
        self.xpos     = xpos
        self.ypos     = ypos
        self.attitude = attitude
        
        self.console_print("Position initialize at: pos=", [ self.xpos, self.ypos, self.attitude])

    def set_command(self, data_to_send):
        self.speed,  self.direction, self.rotation = data_to_send

    def send_command(self):
        data_to_send = [self.speed,  self.direction, self.rotation]
        
        current_time = time.time()
        delta_time = current_time - self.time_of_last_update
        
        if delta_time >= self.update_freq:
            self.time_of_last_update = current_time
            
            stream_to(data_to_send, self.ser_com)
            self.console_print("data", data_to_send)
    

# code execution for this file        
def main(selected_pattern, selected_ports, rigid_body_ids, plotting=True, debug=True):
    
    # create motive thread
    setup_client()
    
    # create robots
    for port, name in get_ports_dict().items():
        print(port, name)

    formation_size = len(rigid_body_ids)
    
    # initial conditions
    platform1_set, platform2_set = selected_pattern()
    
    x0 = [platform1_set[1][0], platform2_set[1][0]]
    y0 = [platform1_set[2][0], platform2_set[2][0]]
    a0 = [platform1_set[3][0], platform2_set[3][0]]
    
    def get_local_offset():
        anchor = Platform(f"anchor", 0, None, rigid_body_ids[0], debug=False) # TODO: add achor robot mocap info
        anchor.get_location()
        
        return anchor.xpos, anchor.ypos, anchor.attitude 
    
    
    local_offset = get_local_offset()  
    
    #print(f"{local_offset = }")
    offset_patern = [transform_frame(selected_pattern()[0], local_offset), transform_frame(selected_pattern()[1], local_offset)]

    formation = [Platform(f"Robot-{i+1}", i, selected_ports[i], rigid_body_ids[i], xpos=x0[i], ypos=y0[i], attitude=a0[i], transform_set=local_offset ,debug=debug) for i in range(formation_size)]
    
    
    #plotting stuff
    fig, ax = plt.subplots()

    xdata1, ydata1 = [], []
    xdata2, ydata2 = [], []

    ln1,   = plt.plot([], [], 'k-', alpha=0.5)
    mark1, = plt.plot([], [], 'ro', label="Robot 1")
    vec1  = plt.quiver([],[],[],[], width=5e-3)

    ln2,   = plt.plot([], [], 'k-', alpha=0.5)
    mark2, = plt.plot([], [], 'bo', label="Robot 2")
    vec2  = plt.quiver([],[],[],[], width=5e-3)

    vecRel = plt.quiver([],[],[],[], width=5e-3)
    lnRel,   = plt.plot([], [], 'k:', alpha=0.5)

    time_disp = plt.text(-1.45,-1.45,"", fontsize=11) 

    presist_on_repeat = False

    
    def init():
        
        get_range_values = lambda pattern, axis : np.array([min(min(pattern[0][axis]), min(pattern[1][axis])), max(max(pattern[0][axis]), max(pattern[1][axis]))])
        
        x_range = get_range_values(offset_patern, 1) + np.array([-0.25, 0.25])
        y_range = get_range_values(offset_patern, 2) + np.array([-0.25, 0.25])
        
        #print(f'{x_range = }')
        
        ax.set_xlim(*x_range)
        ax.set_ylim(*y_range)
        ax.legend()
        ax.set_aspect('equal')
        return ln1,mark1,ln2,mark2,vec1,vec2,vecRel,lnRel,time_disp

    def update(frame):
        global xdata1, ydata1
        global xdata2, ydata2
        
        # reset on repeat:
        if frame == 0.0 and not presist_on_repeat:
            xdata1, ydata1 = [], []
            xdata2, ydata2 = [], []

        # Motion particle 1
        pos1x, pos1y = formation[0].xpos, formation[0].ypos
        
        xdata1.append(pos1x)
        ydata1.append(pos1y)
        ln1.set_data(xdata1, ydata1)
        mark1.set_data(pos1x, pos1y)
        
        # Motion particle 2
        pos2x, pos2y = formation[1].xpos, formation[1].ypos

        xdata2.append(pos2x)
        ydata2.append(pos2y)
        
        ln2.set_data(xdata2, ydata2)
        mark2.set_data(pos2x, pos2y)
        
        lnRel.set_data([pos1x, pos2x], [pos1y, pos2y])
        
    
        return ln1,mark1,ln2,mark2,vec1,vec2,vecRel,lnRel,time_disp
    
    if plotting:
        ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=50, repeat_delay=0)
        plt.show(block=False)
    
    while True:

        formation[0].start_motion(selected_pattern, repeat=True)
        formation[1].start_motion(selected_pattern, repeat=True)
        
        if plotting:
            plt.pause(1/120)
    

        
if __name__ == "__main__":

    # SWARM SETUP
    motion = calibrate_directions
    selected_ports = ["COM3", "COM4"] #["/dev/cu.usbserial-1440", "/dev/cu.usbserial-1450"]
    rigid_body_ids = [1, 1]

    # EXECUTE
    main(motion, selected_ports, rigid_body_ids, plotting=True, debug=True)
     
        # formation[0].test_command(120) #ROBOT 1
        # formation[1].test_command(120) #ROBOT 2

    print("STOPPED")
                
            


    

    

