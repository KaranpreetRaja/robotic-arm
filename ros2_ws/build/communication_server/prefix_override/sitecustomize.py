import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/install/communication_server'
