import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/test/Desktop/Respeaker/res_turtlebot/install/res_turtlebot'
