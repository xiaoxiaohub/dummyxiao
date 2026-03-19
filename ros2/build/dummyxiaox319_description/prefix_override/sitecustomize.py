import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jiehuang/dummyxiao319/ros2/install/dummyxiaox319_description'
