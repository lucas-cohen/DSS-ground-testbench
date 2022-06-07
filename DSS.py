#/usr/bin/env python3

import numpy as np
import time

from communication.wifi_api import open_serial, stream_to, get_ports_dict
from control.default_dynamics import circle_motion
#from communication.motive_api import <functions> #TODO!


class Platform:    
    def __init__(self, name:str, com_port:str, xpos:float=.0, ypos:float=.0, attitude:float=.0, update_freq:float=0.1, debug=False) -> None:
         self.name = name
         
         # position and orientation
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
         self.ser_com = open_serial(self.com_port)
         
         self.update_freq = update_freq
         self.time_of_last_update = time.time()
         self.debug = debug
         
         self.temp_loop_idx = 0
         
         print(self.name, "on", self.com_port)
    
    
    def console_print(self, name, val):
        if self.debug:
            print(f"{self.name}: {name}={val}")
    
    
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

        time_set,x_set,y_set,a_set = dynamics()
            
        current_time = time.time()
        delta_time = current_time - self.time_of_last_update

        if delta_time >= self.update_freq:
            self.time_of_last_update = current_time
        
            dx = x_set[self.temp_loop_idx] - self.xpos
            dy = y_set[self.temp_loop_idx] - self.ypos
            da = (a_set[self.temp_loop_idx] - self.attitude)%(2*np.pi)
            dt = delta_time
            
            
            required_direction  = np.arctan2(dx, dy) + np.pi/2 - self.attitude
            required_speed      = np.sqrt(dx**2 + dy**2)/dt * 1e3
            required_rotation   = da/dt
            
            data_to_send = [required_speed, required_direction, required_rotation]
            
            stream_to(data_to_send, self.ser_com)
            self.console_print("data", data_to_send)
            
            # hardcoding succes of motion
            self.xpos    = x_set[self.temp_loop_idx]
            self.ypos    = y_set[self.temp_loop_idx]
            self.attitude = a_set[self.temp_loop_idx]
            
            if self.temp_loop_idx == len(time_set)-1:
                self.temp_loop_idx = 0
            else:
                self.temp_loop_idx += 1
                
                
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
def main():
    # create robots
    for port,name in get_ports_dict().items():
        print(port,name)
    
    
    selected_ports = ["/dev/cu.usbserial-141110", "/dev/cu.usbserial-141140"]
    formation_size = len(selected_ports)
    formation = [Platform(f"Robot-{i+1}", selected_ports[i], debug=True) for i in range(formation_size)]
    _, x,y,a, = circle_motion()
    x0,y0,a0, = x[0],y[0],a[0]
        
    formation[0].initalise_position(x0, y0,a0)
    formation[1].initalise_position(x0, y0,a0)
    
    while True:
        

        
        
        formation[0].start_motion(circle_motion, repeat=True)
        formation[1].start_motion(circle_motion, repeat=True)
     
        # formation[0].test_command(120) #ROBOT 1
        # formation[1].test_command(120) #ROBOT 2
    
    
    # # Code for running a single robot
    # device = Platform("Main", "/dev/cu.Bluetooth-Incoming-Port")
    
    # while True:
    #     try:
    #         formation[0].test_command(120) #ROBOT 1
    #         #formation[1].test_command(120) #ROBOT 2
            
    #     except KeyboardInterrupt:
    #         interupt_time = time.time()
    #         formation[0].set_command([0,0,0])
            
    #         while time.time() < interupt_time + 1.2*formation[0].update_freq:
    #             formation[0].send_command()
            
    #         break
    
    print("STOPPED")
                
            


    

    

if __name__ == "__main__":
    main()