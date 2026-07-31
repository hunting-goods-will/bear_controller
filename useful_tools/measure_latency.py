# measure_latency.py
from pybear import Manager
from main_controller.config import PORT, BAUDRATE
import time

bear = Manager.BEAR(port=PORT, baudrate=BAUDRATE)
bear.single_timeout = 0.05
bear.single_try_num = 5

# let it get past any warm-up first
for _ in range(30):
    bear.ping(1)

# now measure real round-trip time on 200 known-good reads
times = []
fail_count = 0
for _ in range(200):
    start = time.perf_counter()
    _, err = bear.ping(1)[0]
    elapsed = time.perf_counter() - start
    if err != 128:
        fail_count += 1
    else:
        times.append(elapsed)

print(f"min={min(times)*1000:.2f}ms  max={max(times)*1000:.2f}ms  avg={sum(times)/len(times)*1000:.2f}ms")
print(f"{fail_count}/200 reads failed or returned a non-clean error code")