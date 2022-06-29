#/usr/bin/env python3

### Importing Libraries ###

# Importing Arduino serial comunication
import serial
import serial.tools.list_ports

import numpy as np

import time


# --- Byte Data handling ---
# TODO: Use actual bytes instead of chars to send data!

# integers
def int_to_bytes(x: int, set_bit_length=64) -> bytes : 
    return x.to_bytes((set_bit_length+7)//8, 'big', signed=True)
    
def int_from_bytes(xbytes: bytes) -> int:
    return int.from_bytes(xbytes, 'big', signed=True)

# String
def str_to_bytes(s: str) -> bytes:
    return bytes(s, 'UTF-8')

def str_from_bytes(sbytes : bytes) -> str:
    return sbytes.decode('UTF-8')

# Float
def float_to_bytes(x: float) -> bytes:
    return # FIXME

def float_from_bytes(xbytes: bytes) -> float:
    return # FIXME


# --- Port information & management ---
def get_ports(inc_lics=False):
    """Get availabe serial ports in list of PortInfo object.
    Args:
        inc_lics (bool, optional): include symlinks. Defaults to False.
    """
    
    ports = serial.tools.list_ports.comports(inc_lics)
    return ports


def get_ports_dict(inc_lics=False):
    ports = get_ports()
    ports_dict = {}
    
    for port in ports:
        ports_dict[port.name] = [port.description]
    
    return ports_dict


def open_serial(com_port, baud=9600, bytesize=8, timeout=0.05, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE):
    #FIXME: ensure ports get closed properly if crash!

    ser = serial.Serial(port=com_port, baudrate=baud, bytesize=bytesize, timeout=timeout, parity=parity, stopbits=stopbits)
    ser.rts = 0
    time.sleep(0.1) # Delay to ensure connection is established

    return ser


# --- Read/Write ops ---
def read(ser):
    while ser.in_waiting > 0:
        recieved_bytes = ser.readline()
        decoded_bytes = recieved_bytes.decode('UTF-8').strip()  # Decode bytes using utf-8 without bouding whitespace
        
        return decoded_bytes
    return None


def write_str(data: str, ser):
    ser.write(str_to_bytes(data))
    ser.flush()

    
def stream_to(data_array, ser, debug=False):
    x,y,z = data_array[0:3]
    generator = f"{x},{y},{z}"                    # 2D gen
    #generator = f"{x}, {y}, {z}, {y}, {p}, {r}"   # 3D gen
    
    write_str(generator, ser)
    
    if debug:
    #    print(f">> {ser.name} | {generator}")
        print('generator: ', generator)
 
        
def stream_from(ser, debug=False):
    raise NotImplementedError #TODO: Implement this
    



# Code for development
def main():
    #arduino = serial.Serial(port='/dev/cu.usbmodem14601', baudrate=115200, timeout=.1)
    for port in get_ports():
        print(port.device, port.description, port.name)

    with open_serial('/dev/cu.usbserial-1440') as robot_ser: #/dev/cu.SLAB_USBtoUART #/dev/cu.usbserial-1410

        print(robot_ser.is_open) 
        time.sleep(0.5)
        i = 0
        
        while True:
            #multipliers = np.array([1,2,3,5,7,11])
            # x,y,z,y,p,r = i*multipliers
            # data = f"{x=}, {y=}, {z=}, {y=}, {p=}, {r=}"
               
            write_str(str(i), robot_ser)
            print(i)
            #stream_to(i*multipliers, robot_ser, debug=True)

            time.sleep(0.1) # FIXME no waiting with sleep in final loop
            
            i += 1
            if i > 1:
                i = 0
        
            # while has_recieved == False:
            #     recieved = read(robot_ser)
            #     if recieved != None:
            #         num = recieved
            #         has_recieved = True
            #         print(num)
            
            # has_recieved = False
            
            # data = read(robot_ser)
            # if data != None and data != "hello world":
            #     print(data)
            


if __name__ == "__main__":
    main()


