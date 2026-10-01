import threading
from multiprocessing import Value

sp_side = 51                     # 串口速度偏置量
lock = threading.Lock()          # 线程锁
fc_last_rx_time = Value("d", 0.0)  # 飞控最后收到有效帧的时间戳
