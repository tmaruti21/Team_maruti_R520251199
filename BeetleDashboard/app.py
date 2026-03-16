"""
Beetle Dashboard - Flask Application
Robot Telemetry + Camera Dashboard

Sensor Data Format (14 comma-separated values):
t_ms,ax,ay,az,gx,gy,gz,temp,left_ticks,right_ticks,tof1,tof2,tof3,tof4

POST sensor data to:
http://localhost:5000/update
"""

from flask import Flask, render_template, jsonify, request, Response
import threading
import time
import requests

app = Flask(__name__)

# ============================================
# CONFIG
# ============================================

ROS_CAMERA_STREAM = "http://127.0.0.1:8080/stream?topic=/camera/color/image_raw&quality=50"

# ============================================
# SENSOR DATA STORAGE
# ============================================

sensor_data = {
    "t_ms": 0,

    "ax": 0.0,
    "ay": 0.0,
    "az": 0.0,

    "gx": 0.0,
    "gy": 0.0,
    "gz": 0.0,

    "temp": 0.0,

    "left_ticks": 0.0,
    "right_ticks": 0.0,

    "tof1": 0,
    "tof2": 0,
    "tof3": 0,
    "tof4": 0,

    "yaw": 0.0,

    "enc_left": 0.0,
    "enc_right": 0.0,
}

data_lock = threading.Lock()
last_update_time = time.time()


# ============================================
# DATA PARSING
# ============================================

def parse_sensor_string(data_string):
    """
    Parse ESP sensor string and update stored data.
    """

    global last_update_time

    try:
        values = data_string.strip().split(',')

        if len(values) != 14:
            print("Invalid packet length")
            return False

        current_time = time.time()
        dt = current_time - last_update_time
        last_update_time = current_time

        with data_lock:

            sensor_data["t_ms"] = int(float(values[0]))

            sensor_data["ax"] = float(values[1])
            sensor_data["ay"] = float(values[2])
            sensor_data["az"] = float(values[3])

            sensor_data["gx"] = float(values[4])
            sensor_data["gy"] = float(values[5])
            sensor_data["gz"] = float(values[6])

            sensor_data["temp"] = float(values[7])

            sensor_data["left_ticks"] = float(values[8])
            sensor_data["right_ticks"] = float(values[9])

            sensor_data["enc_left"] += sensor_data["left_ticks"]
            sensor_data["enc_right"] += sensor_data["right_ticks"]

            sensor_data["tof1"] = int(float(values[10]))
            sensor_data["tof2"] = int(float(values[11]))
            sensor_data["tof3"] = int(float(values[12]))
            sensor_data["tof4"] = int(float(values[13]))

            # integrate gyro Z → yaw
            gz_dps = sensor_data["gz"] * 57.2958
            sensor_data["yaw"] += gz_dps * dt
            sensor_data["yaw"] = sensor_data["yaw"] % 360

        return True

    except Exception as e:
        print("Parse error:", e)
        return False


# ============================================
# ROUTES
# ============================================

@app.route("/")
def index():
    """Serve dashboard page"""
    return render_template("index.html")


@app.route("/data")
def get_data():
    """Return latest telemetry"""
    with data_lock:
        return jsonify(sensor_data)


@app.route("/update", methods=["POST"])
def update_data():
    """Receive telemetry packet from ESP / RPi"""

    data_string = request.get_data(as_text=True)

    if parse_sensor_string(data_string):
        return jsonify({"status": "ok"})
    else:
        return jsonify({"status": "error"}), 400


@app.route("/reset", methods=["POST"])
def reset_position():
    """Reset yaw and encoder position"""

    with data_lock:
        sensor_data["yaw"] = 0.0
        sensor_data["enc_left"] = 0.0
        sensor_data["enc_right"] = 0.0

    return jsonify({"status": "ok"})


# ============================================
# CAMERA STREAM (ROS PROXY)
# ============================================




# ============================================
# MAIN
# ============================================

if __name__ == "__main__":

    print("\n" + "=" * 60)
    print("        BEETLE ROBOT DASHBOARD")
    print("=" * 60)
    print("Dashboard:      http://localhost:5000")
    print("Camera Stream:  http://localhost:5000/stream")
    print("Telemetry API:  http://localhost:5000/data")
    print("Update Data:    POST /update")
    print("=" * 60)

    app.run(
        host="0.0.0.0",
        port=5000,
        debug=True,
        threaded=True
    )