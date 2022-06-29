#/usr/bin/env python3

import numpy as np
import Motive_test_1



"""
Functions for data comunicaiton with the Motive Software
"""
import natnetclient as natnet

def get_body_package_data(selected_motive_body_id):
    ## Implement ith motive API

    xpos, ypos, attitude = None, None, None
    
    for body_data in Motive_test_1.rigid_body_list:
        ID, position_data, quad_data = body_data
        
        if ID == selected_motive_body_id:
            xpos, ypos, _   = position_data
            attitude ,_,_,_  = quad_data #FIXME
            break
        
    return [xpos, ypos, attitude] 
    
    


def setup_client():
    Motive_test_1.run()

def main():
    ip_motive = "192.168.209.81"
    #mci = "224.0.0.1 
    client = natnet.NatClient(client_ip=ip_motive, data_port=1511, comm_port=1510)


if __name__ == "__main__":
    main()