import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/takuto/turtlebot3_ws/src/cmd_vel_joy/install/cmd_vel_joy'
