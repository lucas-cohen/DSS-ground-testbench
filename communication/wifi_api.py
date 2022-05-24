### Importing Libraries ###

# Importing Arduino serial comunication
import serial
import serial.tools.list_ports

import time



def get_ports(inc_lics=False):
    """Get availabe serial ports in list of PortInfo object.
    Args:
        inc_lics (bool, optional): include symlinks. Defaults to False.
    """
    
    ports = serial.tools.list_ports.comports(inc_lics)
    return ports


def select_ports(ports, selected_idx):
    pass



def write_read(x, robot):
    robot.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data_robot = robot.readline()
    return data_robot

def write_read_both(x):
    arduino.write(bytes(x, 'utf-8'))
    robot.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data_arduino = arduino.readline()
    data_robot = robot.readline()
    return data_arduino, data_robot

def wifi_write_read():
    pass

def main():
    #arduino = serial.Serial(port='/dev/cu.usbmodem14601', baudrate=115200, timeout=.1)
    robot = serial.Serial(port='/dev/cu.usbserial-1410', baudrate=9600, timeout=.1) #/dev/cu.SLAB_USBtoUART #/dev/cu.usbserial-1410
    
    while True:
        num = input("Enter a number: ") # Taking input from user
        value_robot = write_read(num, robot)
        
        print(f"{value_robot = }")
        
        
    for port in get_ports():
        print(port.device)

if __name__ == "__main__":
    main()


