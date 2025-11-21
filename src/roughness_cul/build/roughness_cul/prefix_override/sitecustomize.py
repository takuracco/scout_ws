import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/takuto/scout_ws/src/roughness_cul/install/roughness_cul'
