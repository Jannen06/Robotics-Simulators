import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shafi/assignments/Robotics-Simulators/install/webots_sim'
