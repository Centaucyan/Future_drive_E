import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hkit/Future_drive_E/ep_part/ep_ws/install/arduino_cartography'
