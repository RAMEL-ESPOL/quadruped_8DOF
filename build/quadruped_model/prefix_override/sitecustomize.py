import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/julio/Desktop/proyecto_tesis/quadruped_8DOF/install/quadruped_model'
