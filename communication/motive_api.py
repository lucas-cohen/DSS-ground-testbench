#/usr/bin/env python3

"""
Functions for data comunicaiton with the Motive Software
"""
import natnetclient as natnet

def get_body_package_data(motive_body_id):
    ## Implement ith motive API

    raise NotImplementedError #TODO:
    
    return [xpos, ypos, attitude]


def main():
    ip_motive = "192.168.209.81"
    #mci = "224.0.0.1 
    client = natnet.NatClient(client_ip=ip_motive, data_port=1511, comm_port=1510)


if __name__ == "__main__":
    main()