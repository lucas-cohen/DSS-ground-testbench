#/usr/bin/env python3

import numpy as np
import time
from datetime import datetime

import matplotlib; matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from communication.wifi_api import open_serial, stream_to, get_ports_dict
from control.default_dynamics import circle_motion, swarm_circle, calibrate_directions, transform_frame, stay_at_position, swarm_relative_test, swarm_full_test
from communication.motive_api import get_body_package_data, setup_client


class Platform:    
    def __init__(self, name:str, idx, com_port:str,  motive_id:int, control_gains:dict, xpos:float=.0, ypos:float=.0, attitude:float=.0, transform_set=[0,0,0], update_freq:float=0.1, debug=False, max_lin_vel=0.4, max_ang_vel=0.8*np.pi) -> None:
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

        self.ser_com = open_serial(self.com_port)
        self.update_freq = update_freq
        self.time_of_last_update = time.time()
        self.start_time = 0

        self.max_lin_vel = max_lin_vel # m/s
        self.max_ang_vel = max_ang_vel # rad/s

        self.debug = debug

        self.temp_loop_idx = 0
        self.robot_idx = idx


        # Motive API data
        self.motive_id = motive_id

        self.x_gains= control_gains['x'] 
        self.y_gains= control_gains['y']
        self.a_gains= control_gains['a'] 

        self.x_controller = PID(*self.x_gains)
        self.y_controller = PID(*self.y_gains)
        self.a_controller = PID(*self.a_gains)

        self.get_location()
        self.tranform_set = transform_set

        if self.debug:
            print(self.name, "on", self.com_port)

    
    def console_print(self, name, val):
        if self.debug:
            print(self.name, "|",name,val)
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
            #self.get_location()
        
            dx = x_set[self.temp_loop_idx] - self.xpos
            dy = y_set[self.temp_loop_idx] - self.ypos
            dir_angle = np.arctan2(dx,dy) #FIXME: check of total magnitude of velocity exceeds, then deal with it
            
            da_raw = (a_set[self.temp_loop_idx] - self.attitude)
            if abs(da_raw) > np.pi:
                da = (da_raw - np.sign(da_raw)*2*np.pi)%(2*np.pi)
            else:
                da = da_raw
            
            # set max magniutde of deltas
            if self.debug:
                pass
                # self.console_print("Actual pos: : ", [round(self.xpos, 3), round(self.ypos, 3)])
                # self.console_print("Target pos: : ", [round(x_set[self.temp_loop_idx], 3), round(y_set[self.temp_loop_idx],3)])
                # self.console_print("deltas : ", [round(dx, 10), round(dy, 10), round(da, 10)])

            dxmax = np.abs(np.sin(dir_angle)) * self.max_lin_vel * delta_time
            dymax = np.abs(np.cos(dir_angle)) * self.max_lin_vel * delta_time
            damax = self.max_ang_vel * delta_time
            
            if np.abs(dx) > dxmax:
#                self.console_print("Forward step exceeds max: ",[np.abs(dx), dxmax])
                #self.console_print("Forward step exceeds max: ",[np.abs(dx), dxmax])
                dx = np.sign(dx) * dxmax
                
            if np.abs(dy) > dymax:
#                self.console_print("Sideways step exceeds max: ",[np.abs(dy), dymax])
                #self.console_print("Sideways step exceeds max: ",[np.abs(dy), dymax])
                dy = np.sign(dy) * dymax
                
            if np.abs(da) > damax:
#                self.console_print("Attitude step exceeds max: ",[np.abs(da), damax])
                #self.console_print("Attitude step exceeds max: ",[np.abs(da), damax])
                dy = np.sign(da) * damax

            # if np.abs(dx) < 5e-2:
            #     dx = 0
            #
            # if np.abs(dy) < 5e-2:
            #     dy = 0
            #
            # if np.abs(da) < 10 * np.pi / 180:
            #     da = 0
            #
            # if np.abs(dx) < 5e-2:
            #     dx = 0
            #
            # if np.abs(dy) < 5e-2:
            #     dy = 0
            #
            # if np.abs(da) < 5*np.pi/180:
            #     da = 0

            # print if capping is taking place
            dt = delta_time

            required_direction  = (np.arctan2(dx, dy) + np.pi/2 - self.attitude) % (2*np.pi)
            required_speed      = np.sqrt(dx**2 + dy**2)/dt * 1e3
            required_rotation   = da/dt
            
            data_to_send = [required_speed, required_direction, required_rotation]

            if self.debug:
                self.console_print("speed: ", round(required_speed,4))
                self.console_print("data", data_to_send)

            stream_to(data_to_send, self.ser_com)

            
            # hardcoding succes of motion
            self.xpos     = x_set[self.temp_loop_idx]
            self.ypos     = y_set[self.temp_loop_idx]
            self.attitude = a_set[self.temp_loop_idx]
            
            if self.temp_loop_idx == len(time_set)-1:
                self.temp_loop_idx = 0
            else:
                self.temp_loop_idx += 1


    def control_motion(self, dynamics, repeat=False):
        time_set, x_set, y_set, a_set = transform_frame(dynamics(dt=self.update_freq)[self.robot_idx], self.tranform_set)

        current_time = time.time()
        delta_time = current_time - self.time_of_last_update

        if delta_time >= self.update_freq:
            self.time_of_last_update = current_time

            # Update locations
            self.get_location()

            # Computes errors
            ex = x_set[self.temp_loop_idx] - self.xpos
            ey = y_set[self.temp_loop_idx] - self.ypos
            ea_raw = (a_set[self.temp_loop_idx] - self.attitude)
            if abs(ea_raw) > np.pi:
                ea = (ea_raw - np.sign(ea_raw)*2*np.pi) % (2*np.pi)
            else:
                ea = ea_raw

            # Compute control commands
            ux = self.x_controller.control(ex, delta_time)
            uy = self.y_controller.control(ey, delta_time)
            ua = self.a_controller.control(ea, delta_time)

            # Compute required control commands
            required_direction = (np.arctan2(ux, uy) - self.attitude) % (2 * np.pi)
            required_speed = np.sqrt(ux ** 2 + uy ** 2) / delta_time * 1e3
            required_rotation_rate = ua / delta_time

            # Impose maximum control commands (SAFETY FEATURE)
            command_speed = np.sign(required_speed) * min(abs(required_speed), 1e3 * self.max_lin_vel)
            command_rotation_rate = np.sign(required_rotation_rate) * min(abs(required_rotation_rate),
                                                                          self.max_ang_vel)

            # Send control commands to Platform
            data_to_send = [command_speed, required_direction, command_rotation_rate]  # command_rotation] #FIXME!
            stream_to(data_to_send, self.ser_com)

            if self.debug:
                pass
                #self.console_print("data", data_to_send)
                self.console_print("Actual pos: : ", [round(self.xpos, 3), round(self.ypos, 3)])
                self.console_print("Target pos: : ", [round(x_set[self.temp_loop_idx], 3), round(y_set[self.temp_loop_idx],3)])
                #self.console_print("deltas : ", [round(ex, 4), round(ey, 4), round(ea, 4)])
                #self.console_print("commands : ", [round(ux, 4), round(uy, 4), round(ua, 4)])

            # hardcoding succes of motion
            # self.xpos     = x_set[self.temp_loop_idx]
            # self.ypos     = y_set[self.temp_loop_idx]
            # self.attitude = a_set[self.temp_loop_idx]


            # next time step
            if self.temp_loop_idx == len(time_set)-1:
                self.temp_loop_idx = 0
            else:
                self.temp_loop_idx += 1

    def go_to_point(self, point_heading):
        current_time = time.time()
        delta_time = current_time - self.time_of_last_update

        if delta_time >= self.update_freq:
            self.time_of_last_update = current_time

            # update location
            self.get_location()

            # Computes errors
            ex = point_heading[0] - self.xpos
            ey = point_heading[1] - self.ypos
            ea_raw = (point_heading[2] - self.attitude)
            if abs(ea_raw) > np.pi:
                ea = (ea_raw - np.sign(ea_raw)*np.pi) % (2*np.pi)
            else:
                ea = -ea_raw

            # Compute control commands
            ux = self.x_controller.control(ex, delta_time)
            uy = self.y_controller.control(ey, delta_time)
            ua = self.a_controller.control(ea, delta_time)

            # Compute required control commands
            required_direction  = (np.arctan2(ux, uy) - self.attitude) % (2*np.pi)
            required_speed      = np.sqrt(ux**2 + uy**2)/delta_time * 1e3
            required_rotation_rate   = ua/delta_time

            # Impose maximum control commands (SAFETY FEATURE)
            command_speed = np.sign(required_speed) * min(abs(required_speed), 1e3*self.max_lin_vel)
            command_rotation_rate = np.sign(required_rotation_rate) * min(abs(required_rotation_rate), self.max_ang_vel)

            # Send control commands to Platform
            data_to_send = [command_speed, required_direction, command_rotation_rate] #command_rotation] #FIXME!
            stream_to(data_to_send, self.ser_com)

            if self.debug:
                self.console_print("data_to_send [v,r,a]: ", np.round(data_to_send,4))
                self.console_print("position (x,y,a): ", [round(self.xpos,4), round(self.ypos,4),round(self.attitude,3)])

    def get_location(self):
        xpos_data, ypos_data, attitude_data = get_body_package_data(self.motive_id)
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
    

class PID:
    def __init__(self, k_P, k_I, k_D, setpoint=0,):

        # tunning
        self.k_P = k_P
        self.k_I = k_I
        self.k_D = k_D

#        self.setpoint = setpoint

        # running variables (for ID)
        self.last_error = 0
        self.error_sum = 0


    def control(self, error, dt):

        # Compute integral and derivitives
        self.error_sum += error * dt
        d_error = (error-self.last_error)/dt


        P = self.k_P * error
        I = self.k_I * self.error_sum
        D = self.k_D * d_error

        y = P+I+D
        # save current error for next call
        self.last_error = error

        return y


    def reset(self):
        # running variables (for ID)
        self.last_error = 0
        self.error_sum = 0


class Swarm:
    def __init__(self, behaviour_name, formation_com_ports, formation_body_ids, gains, local_offset, initial_positions=[], update_freq:float=0.1, wait_time=0, debug=False, logging=False):
        self.local_offset = local_offset

        self.update_freq = update_freq


        current_time = time.time()
        self.wait_time = wait_time
        self.time_of_last_update = current_time
        self.start_time = current_time + self.wait_time

        self.debug = debug
        self.logging = logging
        self.behaviour_name = behaviour_name

        # create motive thread
        # setup_client()

        for port, name in get_ports_dict().items():
            print(port, name)

        self.formation_size = len(formation_body_ids)

        if len(initial_positions) == 0 :
            # initial positions are real positions

            x0 = np.zeros((self.formation_size,1))
            y0 = np.zeros((self.formation_size,1))
            a0 = np.zeros((self.formation_size,1))

            for i, body_id in enumerate(formation_body_ids):
                current_body = Platform(body_id, i, None, body_id, gains, debug=False) # TODO: add achor robot mocap info
                current_body.get_location()

                x0[i], y0[i], a0[i] = current_body.xpos, current_body.ypos, current_body.attitude
        else:
            # initial positions are as given
            x0, y0, a0 = initial_positions[0], initial_positions[1], initial_positions[2]

        # create final set of platforms
        self.platforms = [Platform(f"Robot-{i+1}", i, formation_com_ports[i], formation_body_ids[i], gains, xpos=x0[i], ypos=y0[i], attitude=a0[i], transform_set=self.local_offset ,debug=debug) for i in range(self.formation_size)]


        if self.logging:
            now = datetime.now()
            timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
            filename = "logs/"+behaviour_name.replace(" ","")+"_"+timestamp+".csv"
            print("Log File: ", filename)
            self.log_file = open(filename, 'a')

            self.log_file.write(behaviour_name+" "+ timestamp + "\n")

            for platform in self.platforms:
                self.log_file.write("\n%s gains: \n" % platform.name)
                self.log_file.write("x = %s \n" % platform.x_gains)
                self.log_file.write("y = %s \n" % platform.y_gains)
                self.log_file.write("a = %s \n" % platform.a_gains)

            #header information
            self.log_file.write("\n\nidx\t,t,\tx,\ty,\ta,\tx_tar,\ty_tar,\ta_tar,\tV_com,\tH_com,\tA_com\n")






    def get_relative_position(self, reference_body_idx, do_update=False):

        locations = np.zeros((self.formation_size,2))

        # update and store all locations
        for i, platform in enumerate(self.platforms):
            if do_update:
                platform.get_location()

            locations[i] = [platform.xpos, platform.ypos]

        reference_location = locations[reference_body_idx]
        relative_locations = locations - reference_location

        return relative_locations


    def get_relative_heading(self, reference_body_idx, do_update=False):

        headings = np.zeros((self.formation_size,1))

        # update and store all locations
        for i, platform in enumerate(self.platforms):
            if do_update:
                platform.get_location()

            headings[i] = [platform.attitude]

        reference_heading = headings[reference_body_idx]
        relative_headings = headings - reference_heading

        return relative_headings

    def get_relative_position_heading(self):
        pass


    def run(self, selected_behaviour):
        current_time = time.time()
        delta_time = current_time - self.time_of_last_update

        if delta_time >= self.update_freq:
            self.time_of_last_update = current_time
            running_time = current_time - self.start_time

            for idx, platform in enumerate(self.platforms):

                # update locations
                platform.get_location()

                # get target locations
                x_setpoint, y_setpoint, a_setpoint = selected_behaviour(self.platforms, idx, running_time)

                # Computes errors
                ex = x_setpoint - platform.xpos
                ey = y_setpoint - platform.ypos
                ea_raw = (a_setpoint - platform.attitude)
                if abs(ea_raw) > np.pi:
                    ea = (ea_raw - 2*np.pi) % (2*np.pi)
                else:
                    ea = ea_raw

                # Compute control commands
                ux = platform.x_controller.control(ex, delta_time)
                uy = platform.y_controller.control(ey, delta_time)
                ua = platform.a_controller.control(ea, delta_time)

                # Compute required control commands
                required_direction  = (np.arctan2(ux, uy) - platform.attitude) % (2*np.pi)
                required_speed      = np.sqrt(ux**2 + uy**2)/delta_time * 1e3
                required_rotation   = ua/delta_time

                # Impose maximum control commands (SAFETY FEATURE)
                command_speed = np.sign(required_speed) * min(abs(required_speed), 1e3 * platform.max_lin_vel)
                command_rotation = np.sign(required_rotation) * min(abs(required_rotation), platform.max_ang_vel)

                # Send control commands to Platform
                data_to_send = [command_speed, required_direction, command_rotation]
                stream_to(data_to_send, platform.ser_com)

                if platform.debug:
                    pass
                    platform.console_print("data send:  ", data_to_send)
                    platform.console_print("Actual pos: ", [round(platform.xpos, 3), round(platform.ypos, 3)])
                    platform.console_print("Target pos: ", [round(x_setpoint, 3), round(y_setpoint,3)])
                    #platform.console_print("deltas : ", [round(ex, 4), round(ey, 4), round(ea, 4)])
                    #platform.console_print("commands : ", [round(ux, 4), round(uy, 4), round(ua, 4)])

                if self.logging:
                    "\n\nidx\t,t,\tx,\ty,\ta,\tx_tar,\ty_tar,\ta_tar,\tV_com,\tH_com,\tA_com\n"
                    # info
                    self.log_file.write(str(platform.name)+",\t")
                    self.log_file.write(str(current_time)+ ",\t")

                    # actual pos
                    self.log_file.write(str(platform.xpos) + ",\t")
                    self.log_file.write(str(platform.ypos) + ",\t")
                    self.log_file.write(str(platform.attitude) + ",\t")

                    # target pos
                    self.log_file.write(str(x_setpoint) + ",\t")
                    self.log_file.write(str(y_setpoint) + ",\t")
                    self.log_file.write(str(a_setpoint) + ",\t")

                    # target pos
                    self.log_file.write(str(command_speed) + ",\t")
                    self.log_file.write(str(required_direction) + ",\t")
                    self.log_file.write(str(command_rotation))
                    self.log_file.write("\n")


        


# code execution for this file
def main(selected_pattern, selected_ports, rigid_body_ids, gains, plotting=True, debug=True):
    
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
    
    def get_local_offset(idx=0):
        anchor = Platform(f"anchor", 0, None, rigid_body_ids[idx], gains, debug=False) # TODO: add achor robot mocap info
        anchor.get_location()
        
        return anchor.xpos, anchor.ypos, anchor.attitude

    local_offset = get_local_offset(0)
    
    #print("offset:", local_offset)
    offset_patern = [transform_frame(selected_pattern()[0], get_local_offset(0)), transform_frame(selected_pattern()[1], get_local_offset(0))]
    # offset_patern = selected_pattern

    formation = [Platform(f"Robot-{i+1}", i, selected_ports[i], rigid_body_ids[i], gains, xpos=x0[i], ypos=y0[i], attitude=a0[i], transform_set=get_local_offset(0) ,debug=debug) for i in range(formation_size)]
    
    
    # plotting stuff
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

        formation[1].control_motion(selected_pattern, repeat=True)
        formation[0].control_motion(selected_pattern, repeat=True)

        formation[0].go_to_point([-2,3,0])
        formation[1].go_to_point([-2.5,3,0])
        
        if plotting:
            plt.pause(1e-12)
            


# code execution for this file
def main_swarm(swarmname, behaviour, selected_ports, rigid_body_ids, gains, debug=True, logging=True):

    # create motive thread
    setup_client()

    # create robots
    for port, name in get_ports_dict().items():
        print(port, name)

    formation_size = len(rigid_body_ids)

    def get_local_offset():
        anchor = Platform(f"anchor", 0, None, rigid_body_ids[0], gains, debug=False) # TODO: add achor robot mocap info
        anchor.get_location()

        return anchor.xpos, anchor.ypos, anchor.attitude

    local_offset = [0,0,0] # get_local_offset()

    #formation = [Platform(f"Robot-{i+1}", i, selected_ports[i], rigid_body_ids[i], gains, xpos=x0[i], ypos=y0[i], attitude=a0[i], transform_set=local_offset ,debug=debug) for i in range(formation_size)]
    formation = Swarm(swarmname, selected_ports, rigid_body_ids, gains, local_offset, update_freq=0.1, debug=debug, logging=logging)


    while True:
        try:
            formation.run(behaviour)
        except:

            formation.log_file.close()
            print("File Closed")
            quit()


if __name__ == "__main__":

    # SWARM SETUP
    motion = stay_at_position
    behaviour = swarm_full_test
    selected_ports = ["COM4", "COM3"] #["/dev/cu.usbserial-1440", "/dev/cu.usbserial-1450"]
    rigid_body_ids = [1, 2]
    
    gains={ 'x' : [0.5, 0.009, 0.030], #FIXME: TUNE
            'y' : [0.5, 0.009, 0.030],
            'a' : [0.25, 0.003, 0.017]}

    # EXECUTE
    main_swarm("No Rotation Demo", behaviour, selected_ports, rigid_body_ids, gains, debug=True, logging=True)
    #main(motion, selected_ports, rigid_body_ids, gains, plotting=False, debug=True)
        # formation[0].test_command(120) #ROBOT 1
        # formation[1].test_command(120) #ROBOT 2


    print("STOPPED")
                
            


    

    

