"""
Serial Reader for Beetle Dashboard
Reads sensor data from ESP32 via Serial and sends to Flask app.

Usage:
    python serial_reader.py [PORT] [BAUD]
    
    Default: /dev/ttyUSB0 115200

Expected ESP32 output format:
    DATA:t_ms,ax,ay,az,gx,gy,gz,temp,left_ticks,right_ticks
"""

import serial
import serial.tools.list_ports
import requests
import sys
import time
import argparse

# ============================================
# CONFIGURATION
# ============================================

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
FLASK_URL = "http://localhost:5000/update"
DATA_PREFIX = "DATA:"
EXPECTED_VALUES = 14

# ============================================
# SERIAL READER
# ============================================

def read_serial_and_send(port, baud):
    """
    Read serial data from ESP32 and send to Flask server.
    """
    print(f"\n{'='*60}")
    print("  BEETLE SERIAL READER")
    print(f"{'='*60}")
    print(f"  Serial Port:  {port}")
    print(f"  Baud Rate:    {baud}")
    print(f"  Flask Server: {FLASK_URL}")
    print(f"  Expected CSV: {EXPECTED_VALUES} values")
    print(f"  Format:       t_ms,ax,ay,az,gx,gy,gz,temp,")
    print(f"                left_ticks,right_ticks,tof1-4")
    print(f"{'='*60}\n")
    
    # Open serial connection
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"✓ Connected to {port}")
        time.sleep(2)  # Wait for ESP32 to reset
        
        # Clear any startup messages
        ser.reset_input_buffer()
        print("✓ Listening for sensor data...\n")
        
    except serial.SerialException as e:
        print(f"✗ Error opening serial port: {e}")
        print("\nAvailable ports:")
        try:
            ports = list(serial.tools.list_ports.comports())
            for p in ports:
                print(f"  - {p.device}: {p.description}")
        except:
            print("  (Install pyserial to list ports)")
        return
    
    # Stats
    packets_sent = 0
    packets_failed = 0
    last_status_time = time.time()
    
    try:
        while True:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8').strip()
                except UnicodeDecodeError:
                    continue
                
                if not line:
                    continue
                
                # Check if line is a DATA line
                data = None
                
                if line.startswith(DATA_PREFIX):
                    data = line[len(DATA_PREFIX):]
                elif "," in line:
                    parts = line.split(',')
                    if len(parts) >= EXPECTED_VALUES:
                        data = line
                
                # Print non-data lines (debug/status from ESP32)
                if data is None:
                    print(f"[ESP32] {line}")
                    continue

                # Validate column count
                parts = data.split(',')
                
                if len(parts) < EXPECTED_VALUES:
                    while len(parts) < EXPECTED_VALUES:
                        parts.append("0")
                    data = ",".join(parts)
                elif len(parts) > EXPECTED_VALUES:
                    parts = parts[:EXPECTED_VALUES]
                    data = ",".join(parts)
                
                # Send to Flask
                try:
                    response = requests.post(FLASK_URL, data=data, timeout=1)
                    if response.status_code == 200:
                        packets_sent += 1
                    else:
                        packets_failed += 1
                        print(f"✗ Server error: {response.status_code}")
                        print(f"  Payload: {data}")
                except requests.exceptions.RequestException as e:
                    packets_failed += 1
                    if packets_failed % 10 == 1:
                        print(f"✗ Connection error: {e}")
                
                # Print status every 5 seconds
                if time.time() - last_status_time >= 5:
                    print(f"[Status] Sent: {packets_sent} | Failed: {packets_failed}")
                    last_status_time = time.time()
                    
    except KeyboardInterrupt:
        print(f"\n\n{'='*60}")
        print("  Session ended by user")
        print(f"  Total sent: {packets_sent} | Failed: {packets_failed}")
        print(f"{'='*60}\n")
    finally:
        ser.close()


# ============================================
# MAIN
# ============================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Beetle Dashboard Serial Reader")
    parser.add_argument("port", nargs="?", default=DEFAULT_PORT,
                        help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("baud", nargs="?", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--flask-url", default=FLASK_URL,
                        help=f"Flask server URL (default: {FLASK_URL})")
    
    args = parser.parse_args()
    
    if args.flask_url != FLASK_URL:
        FLASK_URL = args.flask_url
    
    read_serial_and_send(args.port, args.baud)
