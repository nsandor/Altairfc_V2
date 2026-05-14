import pigpio
import time

PIN = 18  # BCM pin number
INTERVAL = 0.5  # seconds

pi = pigpio.pi()

if not pi.connected:
        print("Failed to connect to pigpiod")
        exit(1)

        pi.set_mode(PIN, pigpio.OUTPUT)

        print(f"Blinking GPIO {PIN} every {INTERVAL}s, Ctrl+C to stop")

        try:
                while True:
                        pi.write(PIN, 1)
                                time.sleep(INTERVAL)
                                        pi.write(PIN, 0)
                                                time.sleep(INTERVAL)
        except KeyboardInterrupt:
                pass
        finally:
                pi.write(PIN, 0)
                        pi.stop()
                        print("Done")
