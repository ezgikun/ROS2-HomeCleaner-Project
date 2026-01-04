import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ezgialtiok/homecleaner_ws/install/homecleaner_bot'
