import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/takuto/scout_ws/src/install/cmd_vel_joy'
