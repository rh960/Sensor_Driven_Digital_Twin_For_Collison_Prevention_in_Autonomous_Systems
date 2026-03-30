import sys
import os

# Get the parent directory of the current script
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# Add the parent directory to the Python path
sys.path.append(parent_dir)

import utility.udp_real_time_vis

port = 9575
num_rows = 1
num_cols = 1
plots = [(0, 0, "Range", "Doppler")]


# 3 configration with different suffixes are provided for different ranges (2m, 5m and 10m).
# Uncomment (removing the "#") the one you want to use and comment (adding a "#") the rest, then restart the example.
# Note that it is the max unambiguous range instead of the max detection range.

# cfg_dir = "../radar_config/config_3rx_2m"
cfg_dir = "../radar_config/config_3rx_5m"
# cfg_dir = "../radar_config/config_3rx_10m"
num_angle_bins = 2
filename = None        # set None to disable saving data to file
utility.udp_real_time_vis.main(port, cfg_dir, num_rows, num_cols, plots, num_angle_bins, filename=filename)

