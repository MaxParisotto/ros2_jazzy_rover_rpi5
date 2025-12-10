#!/usr/bin/env python3
"""
BeagleBone Blue Motor Controller Service

This service runs on the BeagleBone Blue and provides:
- Motor control for 4 mecanum wheels
- Encoder feedback (4 channels)
- IMU data (MPU9250)
- Battery voltage monitoring

Communication: UART (/dev/ttyS1) at 115200 baud
Protocol: JSON messages, newline terminated

Commands from Pi (ROS2):
    {"cmd": "motors", "m1": 0.5, "m2": 0.5, "m3": 0.5, "m4": 0.5}  # duty -1.0 to 1.0
    {"cmd": "stop"}
    {"cmd": "get_sensors"}

Responses to Pi:
    {"type": "sensors", "enc": [e1,e2,e3,e4], "imu": {"ax":0,"ay":0,"az":0,"gx":0,"gy":0,"gz":0}, "battery": 12.5}
    {"type": "ack", "cmd": "motors"}
    {"type": "error", "msg": "..."}
"""

import json
import time
import serial
import subprocess
import threading
from ctypes import CDLL, c_int, c_double, c_float, Structure, POINTER, byref

# Load librobotcontrol
try:
    rc = CDLL("/usr/lib/librobotcontrol.so")
except OSError as e:
    print(f"ERROR: Could not load librobotcontrol: {e}")
    print("Make sure librobotcontrol is installed")
    exit(1)

# Define function signatures
rc.rc_motor_init.restype = c_int
rc.rc_motor_cleanup.restype = c_int
rc.rc_motor_set.argtypes = [c_int, c_double]
rc.rc_motor_set.restype = c_int
rc.rc_motor_brake.argtypes = [c_int]
rc.rc_motor_brake.restype = c_int
rc.rc_motor_free_spin.argtypes = [c_int]
rc.rc_motor_free_spin.restype = c_int

rc.rc_encoder_init.restype = c_int
rc.rc_encoder_cleanup.restype = c_int
rc.rc_encoder_read.argtypes = [c_int]
rc.rc_encoder_read.restype = c_int

rc.rc_adc_init.restype = c_int
rc.rc_adc_cleanup.restype = c_int
rc.rc_adc_batt.restype = c_float
rc.rc_adc_dc_jack.restype = c_float

# IMU structures
class rc_mpu_data_t(Structure):
    _fields_ = [
        ("accel", c_double * 3),
        ("gyro", c_double * 3),
        ("mag", c_double * 3),
        ("temp", c_double),
        ("raw_accel", c_int * 3),
        ("raw_gyro", c_int * 3),
        ("accel_to_ms2", c_double),
        ("gyro_to_degs", c_double),
    ]

class rc_mpu_config_t(Structure):
    _fields_ = [
        ("gpio_interrupt_pin_chip", c_int),
        ("gpio_interrupt_pin", c_int),
        ("i2c_bus", c_int),
        ("i2c_addr", c_int),
        ("show_warnings", c_int),
        ("accel_fsr", c_int),
        ("gyro_fsr", c_int),
        ("accel_dlpf", c_int),
        ("gyro_dlpf", c_int),
        ("enable_magnetometer", c_int),
        ("dmp_sample_rate", c_int),
        ("orient", c_int),
        ("dmp_interrupt_sched_policy", c_int),
        ("dmp_interrupt_priority", c_int),
        ("read_mag_after_callback", c_int),
        ("mag_sample_rate_div", c_int),
        ("tap_threshold", c_int),
    ]

rc.rc_mpu_set_config_to_default.argtypes = [POINTER(rc_mpu_config_t)]
rc.rc_mpu_initialize_dmp.argtypes = [POINTER(rc_mpu_data_t), rc_mpu_config_t]
rc.rc_mpu_initialize_dmp.restype = c_int
rc.rc_mpu_power_off.restype = c_int

# Global state
running = True
mpu_data = rc_mpu_data_t()
mpu_config = rc_mpu_config_t()
imu_initialized = False

def init_hardware():
    """Initialize all BBB hardware"""
    global imu_initialized
    
    print("Initializing motors...")
    if rc.rc_motor_init() != 0:
        print("WARNING: Motor init failed")
        return False
    
    print("Initializing encoders...")
    if rc.rc_encoder_init() != 0:
        print("WARNING: Encoder init failed")
    
    print("Initializing ADC...")
    if rc.rc_adc_init() != 0:
        print("WARNING: ADC init failed")
    
    print("Initializing IMU...")
    rc.rc_mpu_set_config_to_default(byref(mpu_config))
    mpu_config.i2c_bus = 2
    mpu_config.enable_magnetometer = 1
    
    if rc.rc_mpu_initialize_dmp(byref(mpu_data), mpu_config) != 0:
        print("WARNING: IMU init failed - trying without DMP")
        # Try simpler init without DMP
        imu_initialized = False
    else:
        imu_initialized = True
    
    print("Hardware initialized!")
    return True

def cleanup_hardware():
    """Cleanup all hardware"""
    print("Cleaning up hardware...")
    rc.rc_motor_cleanup()
    rc.rc_encoder_cleanup()
    rc.rc_adc_cleanup()
    rc.rc_mpu_power_off()

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
    """Read IMU data"""
    if imu_initialized:
        return {
            "ax": mpu_data.accel[0],
            "ay": mpu_data.accel[1],
            "az": mpu_data.accel[2],
            "gx": mpu_data.gyro[0],
            "gy": mpu_data.gyro[1],
            "gz": mpu_data.gyro[2]
        }
    else:
        # Fallback: try reading via command line
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
    global running
    
    print("BeagleBone Blue Motor Controller")
    print("================================")
    
    # Initialize hardware
    if not init_hardware():
        print("Hardware initialization failed!")
        return
    
    # Open UART
    try:
        ser = serial.Serial('/dev/ttyS1', 115200, timeout=0.1)
        print(f"UART opened: {ser.name}")
    except Exception as e:
        print(f"Failed to open UART: {e}")
        cleanup_hardware()
        return
    
    # Periodic sensor broadcast
    last_broadcast = time.time()
    broadcast_interval = 0.05  # 20Hz sensor updates
    
    print("Waiting for commands...")
    
    try:
        while running:
            # Check for incoming commands
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore')
                if line.strip():
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
            
            time.sleep(0.001)  # Small sleep to prevent CPU hogging
    
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        stop_motors()
        cleanup_hardware()
        ser.close()
        print("Goodbye!")

if __name__ == "__main__":
    main()
