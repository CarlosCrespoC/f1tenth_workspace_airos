import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jetson/f1tenth2_ws/src/f1tenth_workspace_airos/install/FTG_node'
