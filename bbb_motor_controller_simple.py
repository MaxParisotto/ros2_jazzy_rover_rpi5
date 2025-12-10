#!/usr/bin/env python3
"""
BeagleBone Blue Motor Controller Service (Simplified)

Communication: UART (/dev/ttyS1) at 115200 baud
Protocol: JSON messages, newline terminated
"""

import json
import time
import serial
import subprocess
from ctypes import CDLL, c_int, c_double, c_float

# Load librobotcontrol
rc = CDLL("/usr/lib/librobotcontrol.so")

# Motor functions
rc.rc_motor_init.restype = c_int
rc.rc_motor_cleanup.restype = c_int
rc.rc_motor_set.argtypes = [c_int, c_double]
rc.rc_motor_set.restype = c_int
rc.rc_motor_brake.argtypes = [c_int]
rc.rc_motor_brake.restype = c_int

# Encoder functions
rc.rc_encoder_init.restype = c_int
rc.rc_encoder_cleanup.restype = c_int
rc.rc_encoder_read.argtypes = [c_int]
rc.rc_encoder_read.restype = c_int

# ADC functions
rc.rc_adc_init.restype = c_int
rc.rc_adc_cleanup.restype = c_int
rc.rc_adc_batt.restype = c_double
rc.rc_adc_dc_jack.restype = c_double


def init_hardware():
    """Initialize all BBB hardware"""
    print("Initializing motors...", flush=True)
    if rc.rc_motor_init() != 0:
        print("WARNING: Motor init failed")
    
    print("Initializing encoders...", flush=True)
    if rc.rc_encoder_init() != 0:
        print("WARNING: Encoder init failed")
    
    print("Initializing ADC...", flush=True)
    if rc.rc_adc_init() != 0:
        print("WARNING: ADC init failed")
    
    print("Hardware initialized!", flush=True)
    return True


def cleanup_hardware():
    """Cleanup all hardware"""
    print("Cleaning up hardware...", flush=True)
    rc.rc_motor_cleanup()
    rc.rc_encoder_cleanup()
    rc.rc_adc_cleanup()


def set_motors(m1, m2, m3, m4):
    """Set motor duty cycles (-1.0 to 1.0)"""
    rc.rc_motor_set(1, c_double(max(-1.0, min(1.0, m1))))
    rc.rc_motor_set(2, c_double(max(-1.0, min(1.0, m2))))
    rc.rc_motor_set(3, c_double(max(-1.0, min(1.0, m3))))
    rc.rc_motor_set(4, c_double(max(-1.0, min(1.0, m4))))


def stop_motors():
    """Stop all motors with brake"""
    for i in range(1, 5):
        rc.rc_motor_brake(i)


def get_encoders():
    """Read all 4 encoder channels"""
    return [
        rc.rc_encoder_read(1),
        rc.rc_encoder_read(2),
        rc.rc_encoder_read(3),
        rc.rc_encoder_read(4)
    ]


def get_imu():
    """Read IMU data via command line (simplified)"""
    # For now, return zeros - can add proper IMU reading later
    return {"ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0}


def get_battery():
    """Read battery voltage"""
    return float(rc.rc_adc_batt())


def get_all_sensors():
    """Get all sensor data"""
    return {
        "type": "sensors",
        "enc": get_encoders(),
        "imu": get_imu(),
        "battery": get_battery(),
        "timestamp": time.time()
    }


def process_command(cmd_str):
    """Process a command from the Pi and return response"""
    try:
        cmd = json.loads(cmd_str.strip())
        
        if cmd.get("cmd") == "motors":
            m1 = cmd.get("m1", 0)
            m2 = cmd.get("m2", 0)
            m3 = cmd.get("m3", 0)
            m4 = cmd.get("m4", 0)
            set_motors(m1, m2, m3, m4)
            return {"type": "ack", "cmd": "motors"}
        
        elif cmd.get("cmd") == "stop":
            stop_motors()
            return {"type": "ack", "cmd": "stop"}
        
        elif cmd.get("cmd") == "get_sensors":
            return get_all_sensors()
        
        elif cmd.get("cmd") == "ping":
            return {"type": "pong", "timestamp": time.time()}
        
        else:
            return {"type": "error", "msg": f"Unknown command: {cmd.get('cmd')}"}
    
    except json.JSONDecodeError as e:
        return {"type": "error", "msg": f"Invalid JSON: {str(e)}"}
    except Exception as e:
        return {"type": "error", "msg": str(e)}


def main():
    print("BeagleBone Blue Motor Controller", flush=True)
    print("================================", flush=True)
    
    # Initialize hardware
    if not init_hardware():
        print("Hardware initialization failed!")
        return
    
    # Open UART
    try:
        ser = serial.Serial('/dev/ttyS1', 115200, timeout=0.1)
        print(f"UART opened: {ser.name}", flush=True)
    except Exception as e:
        print(f"Failed to open UART: {e}")
        cleanup_hardware()
        return
    
    # Periodic sensor broadcast
    last_broadcast = time.time()
    broadcast_interval = 0.05  # 20Hz sensor updates
    
    print("Waiting for commands...", flush=True)
    
    try:
        while True:
            # Check for incoming commands
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore')
                if line.strip():
                    print(f"Received: {line.strip()}", flush=True)
                    response = process_command(line)
                    ser.write((json.dumps(response) + '\n').encode())
                    ser.flush()
            
            # Periodic sensor broadcast
            now = time.time()
            if now - last_broadcast >= broadcast_interval:
                sensors = get_all_sensors()
                ser.write((json.dumps(sensors) + '\n').encode())
                ser.flush()
                last_broadcast = now
            
            time.sleep(0.001)
    
    except KeyboardInterrupt:
        print("\nShutting down...", flush=True)
    finally:
        stop_motors()
        cleanup_hardware()
        ser.close()
        print("Goodbye!", flush=True)


if __name__ == "__main__":
    main()
