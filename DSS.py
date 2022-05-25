import numpy as np

from communication.wifi_api import open_serial, stream_to, stream_from


class Device:    
    def __init__(self, name, xpos:float, ypos:float, attitude:float, com_port:str) -> None:
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
         self.ser_com = RF.open_serial(self.com_port)
        
        