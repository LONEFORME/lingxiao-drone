import threading
from multiprocessing import Value

sp_side = 51                     # 串口速度偏置量
lock = threading.Lock()          # 线程锁
fc_last_rx_time = Value("d", 0.0)  # 飞控最后收到有效帧的时间戳
fc_last_rx_monotonic = Value("d", 0.0)  # 飞控关键帧接收时刻（单调时钟，供控制新鲜度判断）
fc_frame_counter = Value("L", 0)  # 已接收的有效关键帧计数（供解锁门禁与原子快照使用）
