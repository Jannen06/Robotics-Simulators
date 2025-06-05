import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jannen/Documents/git/Robotics-Simulators/src/webots_sim/install/webots_sim'
