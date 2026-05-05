import serial
import sys

try:
    # Use a longer timeout just in case
    with serial.Serial('/dev/ttyACM1', 115200, timeout=10) as ser:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore')
            if line:
                sys.stdout.write(line)
                sys.stdout.flush()
            if 'TEST_COMPLETE' in line:
                sys.exit(0)
except Exception as e:
    print(f"Monitor error: {e}")
    sys.exit(1)
