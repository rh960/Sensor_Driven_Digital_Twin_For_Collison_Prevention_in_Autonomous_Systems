import sys
import os

# Get the parent directory of the current script
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# Add the parent directory to the Python path
sys.path.append(parent_dir)

import utility.udp_streaming

# IP_addr = "192.168.236.1"
IP_addr = "127.0.0.1"    # set the IP to receive data
port = 9575
cfg_dir = "../radar_config/config_3rx_5m"
filename = None        # set None to disable saving data to file
utility.udp_streaming.main(IP_addr, port, cfg_dir, filename)