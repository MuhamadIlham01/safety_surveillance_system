import cv2
import numpy as np
import paho.mqtt.client as mqtt
from flask import Flask, render_template, Response, request
import threading
from datetime import datetime
import time
from flask import jsonify
import json
from ultralytics import YOLO
import os
import uuid
import threading

app = Flask(__name__)

# MQTT Settings
MQTT_BROKER = "test.mosquitto.org"
MQTT_PORT = 1883
MQTT_TOPICS = [
    "surveillance/temperature",
    "surveillance/humidity",
    "surveillance/gas",
    "surveillance/distance",
    "surveillance/motion",
    "surveillance/alert"
]

# Stream Settings
ESP32_CAM_URL = "http://10.153.103.22:81/stream"
MAX_RETRIES = 5
RETRY_DELAY = 2

# Sensor Data with thresholds
sensor_data = {
    "temperature": 0,
    "humidity": 0,
    "gas": 0,
    "distance": 0,
    "motion": False,
    "alert": "SAFE",
    "weapon_detected": False,
    "last_update": "",
    "thresholds": {
        "temperature": 35,    # °C
        "gas": 1500,         # ppm
        "distance": 50,      # cm
        "motion": True,       # boolean
        "weapon": True        # boolean
    }
}

# Load YOLOv8 model
model = YOLO("best.pt")  # or "best.pt" for custom model

# Global variables
cap = None
latest_annotated_frame = None
frame_lock = threading.Lock()
IMAGE_SAVE_DIR = "static/captures"
os.makedirs(IMAGE_SAVE_DIR, exist_ok=True)

# MQTT Callbacks
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT Broker with result code "+str(rc))
    for topic in MQTT_TOPICS:
        client.subscribe(topic)

def on_message(client, userdata, msg):
    topic = msg.topic.split("/")[-1]
    value = msg.payload.decode()
    
    if topic == "temperature":
        sensor_data["temperature"] = float(value)
        if float(value) >= sensor_data["thresholds"]["temperature"]:
            sensor_data["alert"] = "WARNING"
    elif topic == "humidity":
        sensor_data["humidity"] = float(value)
    elif topic == "gas":
        sensor_data["gas"] = int(value)
        if int(value) >= sensor_data["thresholds"]["gas"]:
            sensor_data["alert"] = "WARNING"
    elif topic == "distance":
        sensor_data["distance"] = float(value)
        if float(value) <= sensor_data["thresholds"]["distance"]:
            sensor_data["alert"] = "WARNING"
    elif topic == "motion":
        sensor_data["motion"] = bool(int(value))
        if bool(int(value)) and sensor_data["thresholds"]["motion"]:
            sensor_data["alert"] = "WARNING"
    elif topic == "alert":
        sensor_data["alert"] = value
    
    sensor_data["last_update"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

# Setup MQTT Client
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_thread = threading.Thread(target=mqtt_client.loop_forever)
mqtt_thread.daemon = True
mqtt_thread.start()

def init_video_stream():
    global cap
    retry_count = 0
    while retry_count < MAX_RETRIES:
        try:
            cap = cv2.VideoCapture(ESP32_CAM_URL)
            if cap.isOpened():
                print("Stream opened successfully.")
                return True
            else:
                raise ConnectionError("Could not open video stream")
        except Exception as e:
            print(f"Connection error: {e}")
            retry_count += 1
            if retry_count < MAX_RETRIES:
                print(f"Attempting to reconnect ({retry_count}/{MAX_RETRIES})...")
                time.sleep(RETRY_DELAY)
    return False

def detect_objects():
    global cap, latest_annotated_frame
    
    if not init_video_stream():
        print("Failed to initialize video stream after multiple attempts")
        return
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Frame read error - attempting to reconnect...")
                if not init_video_stream():
                    break
                continue
                
            try:
                # Perform object detection
                results = model(frame, verbose=False)
                
                # Check for weapons
                weapon_detected = False
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        if box.conf > 0.5:  # Confidence threshold
                            cls = int(box.cls)
                            label = model.names[cls]
                            if label.lower() in ['handgun', 'knife']:
                                weapon_detected = True
                                break
                    if weapon_detected:
                        break
                
                # Update sensor data
                if weapon_detected != sensor_data["weapon_detected"]:
                    sensor_data["weapon_detected"] = weapon_detected
                    if weapon_detected and sensor_data["thresholds"]["weapon"]:
                        sensor_data["alert"] = "WARNING"
                        mqtt_client.publish("surveillance/alert", "WARNING")
                    else:
                        sensor_data["alert"] = "SAFE"
                        mqtt_client.publish("surveillance/alert", "SAFE")
                    sensor_data["last_update"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                
                # Store the latest frame with detections
                with frame_lock:
                    latest_annotated_frame = results[0].plot()
                
                ret, buffer = cv2.imencode('.jpg', latest_annotated_frame)
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                
            except Exception as e:
                print(f"Frame processing error: {e}")
                continue
                
    except Exception as e:
        print(f"Error in detect_objects: {e}")
    finally:
        if cap is not None:
            cap.release()
        print("Video stream ended.")

@app.route('/video_feed')
def video_feed():
    return Response(detect_objects(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/sensor_data')
def get_sensor_data():
    return jsonify(sensor_data)

@app.route('/sensor_stream')
def sensor_stream():
    def event_stream():
        last_data = ""
        while True:
            current_data = json.dumps(sensor_data)
            if current_data != last_data:
                yield f"data: {current_data}\n\n"
                last_data = current_data
            time.sleep(0.5)
    
    return Response(event_stream(), mimetype="text/event-stream")

@app.route('/capture', methods=['POST'])
def handle_capture():
    try:
        with frame_lock:
            if latest_annotated_frame is None:
                return jsonify({"status": "error", "message": "No frame available"}), 400
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.jpg"
            save_path = os.path.join(IMAGE_SAVE_DIR, filename)
            
            cv2.imwrite(save_path, latest_annotated_frame)
            
            return jsonify({
                "status": "success",
                "message": "Image captured successfully",
                "path": f"/static/captures/{filename}"
            })
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/')
def index():
    return render_template('index.html', data=sensor_data)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
