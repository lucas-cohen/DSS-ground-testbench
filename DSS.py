import numpy as np
import time

from communication.wifi_api import open_serial, stream_to, get_ports_dict

class Platform:    
    def __init__(self, name:str, com_port:str, xpos:float=.0, ypos:float=.0, attitude:float=.0) -> None:
         # TODO: MAKE INIT
         # 
         self.name = name
         
         # position and orientation
         self.xpos      = xpos
         self.ypos      = ypos
         self.pos = np.array([self.xpos, self.ypos])
         
         self.attitude   = attitude
         
         # Create serial link to device
         self.com_port = com_port
         self.ser_com = open_serial(self.com_port)
    
    def test_motion(self):
        magnitude = 100.0
        dir_range = np.arange(0,2*np.pi, 4*np.pi/180)
        rotation = 0.0

        for direction in dir_range:
            data_to_send = [magnitude, direction, rotation]
            stream_to(data_to_send, self.ser_com, debug=True)
            time.sleep(0.1)







# code execution for this file        
def main():
    # create robots
    for port,name in get_ports_dict().items():
        print(port,name)
    
    
    device = Platform("Main", "/dev/cu.usbmodem14601")
    while True:
        device.test_motion()
    
        
        
    

    

if __name__ == "__main__":
    main()