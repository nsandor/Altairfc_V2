from drivers.vesc_interface import VESCObject
import time

PORT = "/dev/ttyACM1"
RPM = 3400
DURATION = 3

vesc = VESCObject(PORT)

try:
    print(f"Sending {RPM} RPM to {PORT}")
    t0 = time.time()
    while time.time() - t0 < DURATION:
        vesc.set_rpm(RPM)
        time.sleep(0.05)

    print("Stopping...")
    for _ in range(20):
        vesc.set_rpm(0)
        time.sleep(0.05)

finally:
    try:
        vesc.set_rpm(0)
    except Exception:
        pass

print("Done.")
