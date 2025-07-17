import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user/data/drones_ship_ws/install/ros_gz_crazyflie_control'
