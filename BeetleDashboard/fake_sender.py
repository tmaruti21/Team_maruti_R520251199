"""
Fake ESP32 Data Sender for testing the dashboard without hardware.
Simulates sensor data and POSTs it to the Flask server.

Usage:
    1. Start Flask:  python app.py
    2. Run this:     python fake_sender.py

Simulates: MPU6050 + 6 encoders (averaged) + 4 ToF sensors
Format:    t_ms,ax,ay,az,gx,gy,gz,temp,left_ticks,right_ticks,tof1,tof2,tof3,tof4
"""

import requests
import time
import math
import random

FLASK_URL = "http://localhost:5000/update"
INTERVAL = 0.1  # 10 Hz, same as ESP32

def main():
    print("="*50)
    print("  FAKE ESP32 DATA SENDER")
    print("="*50)
    print(f"  Sending to: {FLASK_URL}")
    print(f"  Interval:   {INTERVAL}s (10 Hz)")
    print("  Press Ctrl+C to stop")
    print("="*50 + "\n")

    t = 0  # simulated time in ms
    packets = 0

    try:
        while True:
            t += int(INTERVAL * 1000)

            # --- Simulated MPU6050 ---
            # Slight wobble on accel (gravity on Z + noise)
            ax = random.gauss(0.0, 0.3)
            ay = random.gauss(0.0, 0.3)
            az = random.gauss(9.81, 0.2)

            # Slow rotation on gz (simulates turning)
            gx = random.gauss(0.0, 0.01)
            gy = random.gauss(0.0, 0.01)
            gz = 0.05 * math.sin(t / 5000.0) + random.gauss(0.0, 0.005)

            # Temperature (slowly drifting)
            temp = 25.0 + 2.0 * math.sin(t / 30000.0) + random.gauss(0.0, 0.1)

            # --- Simulated Encoders (averaged) ---
            # Robot moving mostly forward with slight drift
            base_speed = 5.0 + 2.0 * math.sin(t / 10000.0)
            left_ticks = base_speed + random.gauss(0, 0.5)
            right_ticks = base_speed + random.gauss(0, 0.5) + 0.3 * math.sin(t / 8000.0)

            # --- Simulated ToF (mm) ---
            # Distances fluctuating as if objects are nearby
            tof1 = int(200 + 100 * math.sin(t / 4000.0) + random.gauss(0, 10))
            tof2 = int(350 + 150 * math.sin(t / 6000.0) + random.gauss(0, 15))
            tof3 = int(500 + 200 * math.cos(t / 5000.0) + random.gauss(0, 10))
            tof4 = int(180 + 80 * math.cos(t / 3000.0) + random.gauss(0, 8))

            # Clamp ToF to valid range
            tof1 = max(30, min(tof1, 8190))
            tof2 = max(30, min(tof2, 8190))
            tof3 = max(30, min(tof3, 8190))
            tof4 = max(30, min(tof4, 8190))

            # Build CSV
            data = (
                f"{t},"
                f"{ax:.4f},{ay:.4f},{az:.4f},"
                f"{gx:.5f},{gy:.5f},{gz:.5f},"
                f"{temp:.2f},"
                f"{left_ticks:.2f},{right_ticks:.2f},"
                f"{tof1},{tof2},{tof3},{tof4}"
            )

            try:
                response = requests.post(FLASK_URL, data=data, timeout=1)
                packets += 1
                if packets % 50 == 0:  # Print every 5 seconds
                    print(f"[OK] Sent {packets} packets | Latest: {data[:60]}...")
            except requests.exceptions.RequestException as e:
                print(f"[ERR] {e}")

            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print(f"\n\nStopped. Total packets sent: {packets}")

if __name__ == "__main__":
    main()
