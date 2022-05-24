### Importing Libraries ###

# Importing Arduino serial comunication
import re
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


def read(ser):
    
    while ser.in_waiting > 0:
        recieved_bytes = ser.readline()
        decoded_bytes = recieved_bytes.decode('UTF-8').strip()  # Decode bytes using utf-8 without bouding whitespace
        
        return decoded_bytes
    return None


def write(data, ser):
    pass
    



def write_read_both(x):
    arduino.write(bytes(x, 'utf-8'))
    robot.write(bytes(x, 'utf-8'))
    time.sleep(0.1)
    data_arduino = arduino.readline()
    data_robot = robot.readline()
    return data_arduino, data_robot

def wifi_write_read():
    pass

def main():
    #arduino = serial.Serial(port='/dev/cu.usbmodem14601', baudrate=115200, timeout=.1)
    for port in get_ports():
        print(f"{port.device} - {port.description}")
        
            
    with serial.Serial(port='COM6', baudrate=9600, bytesize=8, timeout=.1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE) as robot_ser: #/dev/cu.SLAB_USBtoUART #/dev/cu.usbserial-1410
        robot_ser.setRTS(0)
        print(robot_ser.is_open) 
        while True:
            # num = input("Enter a number: ") # Taking input from user
            # value_robot = write_read(num, robot)
            
            # print(f"{value_robot = }")
            
            data = read(robot_ser)
            
            if data != None:
                print(data)
            
        
        

        
    robot.close()

if __name__ == "__main__":
    main()


