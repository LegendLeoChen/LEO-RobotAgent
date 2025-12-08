#!/usr/bin/env python3
import base64
import io
import cv2
import numpy as np
from flask import Flask, request, jsonify, Response
from ultralytics import YOLO

app = Flask(__name__)

# Initialize YOLO model
model = YOLO("./src/agent/weights/yolov8l-worldv2.pt")
model.set_classes(["person", "vase", "bus", "bookshelf", "desk", "chair",
                   "car", "ladder", "lamp", "door", "tv", "dining table", "bottle"])

# Camera for video stream detection
cap = cv2.VideoCapture(0)

# -------------------------
# Image detection (POST /detect)
# -------------------------
@app.route("/detect", methods=["POST"])
def detect_image():
    """
    Receive a single image (base64 or multipart file) and return detection results in JSON.
    """
    if 'file' in request.files:
        # multipart/form-data uploaded file
        file = request.files['file']
        image = cv2.imdecode(np.frombuffer(file.read(), np.uint8), cv2.IMREAD_COLOR)
    else:
        # base64 encoded image in JSON
        data = request.get_json()
        if data is None or "image" not in data:
            return jsonify({"error": "No image provided"}), 400
        image_data = base64.b64decode(data["image"])
        image = cv2.imdecode(np.frombuffer(image_data, np.uint8), cv2.IMREAD_COLOR)

    # YOLO inference
    results = model.predict(image, verbose=False)
    detections = []
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            label = model.names[cls_id]
            detections.append({
                "label": label,
                "confidence": round(conf, 3),
                "box": [x1, y1, x2, y2],
                "cls": cls_id,
            })

    return jsonify({"status": "ok", "objects": detections})


# -------------------------
# Video stream detection (GET /video)
# -------------------------
def generate_frames():
    """
    Real-time detection using the webcam, returning video stream frames.
    """
    while True:
        success, frame = cap.read()
        if not success:
            break

        results = model.predict(frame, verbose=False)
        annotated_frame = results[0].plot()

        # Convert to JPEG
        ret, buffer = cv2.imencode('.jpg', annotated_frame)
        frame_bytes = buffer.tobytes()

        # Use multipart/x-mixed-replace for video streaming
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route("/video")
def video_feed():
    """
    Video stream endpoint. Open in browser: http://localhost:5001/video
    """
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# -------------------------
# Start service
# -------------------------
if __name__ == "__main__":
    print("🚀 YOLO Flask video detection server started: http://localhost:5001")
    print("📸 Real-time video stream: http://localhost:5001/video")
    print("🖼️  Image detection POST API: /detect")
    app.run(host="0.0.0.0", port=5001, debug=False)
