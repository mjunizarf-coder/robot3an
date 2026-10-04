import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jun/ros2_jazzy/src/simple_mover/install/simple_mover'
