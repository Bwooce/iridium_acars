import serial
import serial.tools.list_ports
import time
import sys

TARGET_VID = 0x1a86
TARGET_PID = 0x55d3
BAUD_RATE = 115200
LOG_FILE = "p4_performance.log"

def find_p4_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == TARGET_VID and port.pid == TARGET_PID:
            return port.device
    return None

def monitor():
    with open(LOG_FILE, "a") as log:
        log.write(f"\n--- MONITOR START: {time.ctime()} ---\n")
        log.flush()
    
    while True:
        port_name = find_p4_port()
        if not port_name:
            time.sleep(2)
            continue
        
        try:
            with serial.Serial(port_name, BAUD_RATE, timeout=0.5) as ser:
                with open(LOG_FILE, "a") as log:
                    log.write(f"Connected to {port_name}\n")
                    log.flush()
                    while True:
                        line = ser.readline()
                        if not line:
                            continue
                        
                        try:
                            decoded = line.decode('utf-8', errors='replace').strip()
                            if decoded:
                                log.write(decoded + "\n")
                                log.flush()
                                if "Rate:" in decoded:
                                    print(decoded, flush=True)
                        except Exception as e:
                            log.write(f"Decode error: {e}\n")
                                
        except Exception as e:
            with open(LOG_FILE, "a") as log:
                log.write(f"Connection error: {e}\n")
                log.flush()
            time.sleep(2)

if __name__ == "__main__":
    monitor()
