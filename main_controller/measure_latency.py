# measure_latency.py
from pybear import Manager
from config import PORT, BAUDRATE
import time

bear = Manager.BEAR(port=PORT, baudrate=BAUDRATE)
bear.single_timeout = 0.05
bear.single_try_num = 5

# let it get past any warm-up first
for _ in range(30):
    bear.ping(1)

# now measure real round-trip time on 20 known-good reads
times = []
for _ in range(200):
    start = time.perf_counter()
    bear.ping(1)
    times.append(time.perf_counter() - start)

print(f"min={min(times)*1000:.2f}ms  max={max(times)*1000:.2f}ms  avg={sum(times)/len(times)*1000:.2f}ms")