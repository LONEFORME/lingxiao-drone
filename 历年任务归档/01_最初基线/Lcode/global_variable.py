import threading
import time
from multiprocessing import Value
sp_side=51 #串口传输速度偏置量
lock=threading.Lock()#线程锁
task_start_sign=Value("b",False)
fc_last_rx_time=Value("d",0.0)  # 飞控串口最后收到有效帧的时间戳