import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zeyadcode_jammy/ros_ws/src/install/roben_ai'
