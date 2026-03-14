"""
Beetle Dashboard - Flask Application
Robot Telemetry Dashboard

Data Format (14 values as comma-separated string, prefixed with DATA:):
t_ms,ax,ay,az,gx,gy,gz,temp,left_ticks,right_ticks,tof1,tof2,tof3,tof4

Send data via POST to /update endpoint.
"""

from flask import Flask, render_template, jsonify, request
import threading
import time

app = Flask(__name__)

# ============================================
# SENSOR DATA STORAGE
# ============================================
sensor_data = {
    # Timestamp from ESP32 (ms)
    't_ms': 0,
    # MPU6050 Linear Acceleration (m/s²)
    'ax': 0.0,
    'ay': 0.0,
    'az': 0.0,
    # MPU6050 Angular Velocity (rad/s)
    'gx': 0.0,
    'gy': 0.0,
    'gz': 0.0,
    # MPU6050 Temperature (°C)
    'temp': 0.0,
    # Averaged encoder delta ticks per interval
    'left_ticks': 0.0,
    'right_ticks': 0.0,
    # 4 ToF Sensors (distances in mm)
    'tof1': 0,
    'tof2': 0,
    'tof3': 0,
    'tof4': 0,
    # Calculated yaw from gz integration
    'yaw': 0.0,
    # Cumulative encoder ticks (for map position tracking)
    'enc_left': 0.0,
    'enc_right': 0.0,
}

# Thread lock for safe data access
data_lock = threading.Lock()

# Last update timestamp for yaw integration
last_update_time = time.time()


# ============================================
# DATA PARSING
# ============================================
def parse_sensor_string(data_string):
    """
    Parse comma-separated sensor data string.
    Format: t_ms,ax,ay,az,gx,gy,gz,temp,left_ticks,right_ticks,tof1,tof2,tof3,tof4
    """
    global sensor_data, last_update_time
    
    try:
        values = data_string.strip().split(',')
        
        if len(values) != 14:
            print(f"Error: Expected 14 values, got {len(values)}")
            return False
        
        # Calculate time delta for yaw integration
        current_time = time.time()
        dt = current_time - last_update_time
        last_update_time = current_time
        
        with data_lock:
            # Timestamp
            sensor_data['t_ms'] = int(float(values[0]))
            
            # MPU6050 - Linear Acceleration (m/s²)
            sensor_data['ax'] = float(values[1])
            sensor_data['ay'] = float(values[2])
            sensor_data['az'] = float(values[3])
            
            # MPU6050 - Angular Velocity (rad/s)
            sensor_data['gx'] = float(values[4])
            sensor_data['gy'] = float(values[5])
            sensor_data['gz'] = float(values[6])
            
            # MPU6050 - Temperature (°C)
            sensor_data['temp'] = float(values[7])
            
            # Averaged encoder delta ticks
            sensor_data['left_ticks'] = float(values[8])
            sensor_data['right_ticks'] = float(values[9])
            
            # Accumulate encoder ticks (for map position tracking)
            sensor_data['enc_left'] += sensor_data['left_ticks']
            sensor_data['enc_right'] += sensor_data['right_ticks']
            
            # 4 ToF Sensors (mm)
            sensor_data['tof1'] = int(float(values[10]))
            sensor_data['tof2'] = int(float(values[11]))
            sensor_data['tof3'] = int(float(values[12]))
            sensor_data['tof4'] = int(float(values[13]))
            
            # Integrate gz to get yaw angle
            # gz is in rad/s, convert to deg/s for yaw display
            gz_dps = sensor_data['gz'] * 57.2958  # rad/s -> deg/s
            sensor_data['yaw'] += gz_dps * dt
            sensor_data['yaw'] = sensor_data['yaw'] % 360
        
        return True
        
    except Exception as e:
        print(f"Parse error: {e}")
        return False


# ============================================
# FLASK ROUTES
# ============================================

@app.route('/')
def index():
    """Serve the main dashboard page."""
    return render_template('index.html')


@app.route('/data')
def get_data():
    """GET - Retrieve current sensor data as JSON."""
    with data_lock:
        return jsonify(sensor_data)


@app.route('/update', methods=['POST'])
def update_data():
    """
    POST - Receive sensor data from ESP/RPi.
    
    Format: t_ms,ax,ay,az,gx,gy,gz,temp,left_ticks,right_ticks,tof1,tof2,tof3,tof4
    """
    try:
        data_string = request.get_data(as_text=True)
        print(f"Received: {data_string}")
        
        if parse_sensor_string(data_string):
            return jsonify({'status': 'ok'}), 200
        else:
            return jsonify({'status': 'error', 'message': 'Parse failed'}), 400
            
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/reset', methods=['POST'])
def reset_position():
    """Reset yaw angle and accumulated encoder counts."""
    global sensor_data
    with data_lock:
        sensor_data['yaw'] = 0.0
        sensor_data['enc_left'] = 0.0
        sensor_data['enc_right'] = 0.0
    return jsonify({'status': 'ok'})


@app.route('/stream')
def video_stream():
    """Camera stream - not implemented."""
    return '', 404


# ============================================
# MAIN
# ============================================

if __name__ == '__main__':
    print("\n" + "="*60)
    print("  BEETLE DASHBOARD")
    print("="*60)
    print("  Dashboard:     http://localhost:5000")
    print("  Data API:      GET  http://localhost:5000/data")
    print("  Update Data:   POST http://localhost:5000/update")
    print("  Reset:         POST http://localhost:5000/reset")
    print("="*60)
    print("\n  Data Format (14 values, comma-separated):")
    print("  t_ms,ax,ay,az,gx,gy,gz,temp,")
    print("  left_ticks,right_ticks,tof1,tof2,tof3,tof4")
    print("\n  Test with curl:")
    print('  curl -X POST -d "1000,0.1,0.2,9.8,0.01,0.02,0.03,25.5,5.33,5.67,120,130,140,150" http://localhost:5000/update')
    print("="*60 + "\n")
    
    app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)
