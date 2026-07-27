from pybear import Manager
from config import PORT, BAUDRATE
import time

bear = Manager.BEAR(port=PORT, baudrate=BAUDRATE)
TEST_ID = 1
successes = 0
trials = 20

for i in range(trials):
    result = bear.ping(TEST_ID)
    value, err = result[0]
    ok = value[0] is not None
    successes += ok
    print(f"Attempt {i+1}: {'OK' if ok else 'TIMEOUT'}")
    time.sleep(0.2)

print(f"\n{successes}/{trials} succeeded")