#/usr/bin/env python3

"""
Functions for data comunicaiton with the Motive Software
"""
import natnetclient as natnet

def get_body_package_data(platform):
    ## Implement ith motive API
    return [platform.xpos, platform.ypos, platform.attitude]


def main():
    ip_motive = "192.168.209.81"
    #mci = "224.0.0.1 
    client = natnet.NatClient(client_ip=ip_motive, data_port=1511, comm_port=1510)


if __name__ == "__main__":
    main()