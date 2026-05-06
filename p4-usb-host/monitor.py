import serial
import sys
import time

try:
    with serial.Serial('/dev/ttyACM1', 115200, timeout=5) as ser:
        end_time = time.time() + 10
        while time.time() < end_time:
            line = ser.readline().decode('utf-8', errors='ignore')
            if line:
                sys.stdout.write(line)
                sys.stdout.flush()
        sys.exit(0)
except Exception as e:
    print(f"Monitor error: {e}")
    sys.exit(1)
