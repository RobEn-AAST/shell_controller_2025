import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zeyadcode_jammy/shell_ws/src/install/ros_compatibility'
