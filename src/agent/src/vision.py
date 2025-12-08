#!/usr/bin/env python3
import numpy as np
import base64, os
import cv2, requests
import json

import rospy
import tf
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates

from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLOWorld
from openai import OpenAI

device_info = {
    "uav": {
        "device": "uav",
        "camera_link": "realsense_camera_link",
        "camerainfo_topic": "/realsense_plugin/camera/color/camera_info",
        "base_link": "base_link",
        "depth": "mm"
    },
    "car": {
        "device": "car",
        "camera_link": "camera_link",
        "camerainfo_topic": "/camera/camera_info",
        "base_link": "base_link",
        "depth": "m"
    },
    "dog": {
        "device": "dog",
        "camera_link": "camera_face",
        "camerainfo_topic": "/camera_face/color/camera_info",
        "base_link": "base",
        "depth": "m"
    }
}


class VisionNode:
    def __init__(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        device_file = os.path.join(current_dir, "../config/vision_device.txt")
        with open(device_file, "r", encoding="utf-8") as f:
            self.device = f.read().strip()          # uav or car or dog
        print(f"device: {self.device}")
        rospy.init_node('vision_node', anonymous=True)

        self.model = YOLOWorld("./src/agent/weights/yolov8l-worldv2.pt").to('cuda')  # World model
        self.model.set_classes(["person", "vase", "bus", "bookshelf", "desk", "chair", "bookshelf", "car", "ladder", "lamp", "door", "tv", "dining table", "bottle"])             # Custom classes
        
        if self.device == "uav":
            rospy.Subscriber("/realsense_plugin/camera/color/image_raw", Image, self.horizon_image_callback)    # UAV horizontal RGB
            rospy.Subscriber("/realsense_plugin/camera/depth/image_raw", Image, self.horizon_depth_callback)    # UAV horizontal depth
            rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_callback)                       # UAV pose
        elif self.device == "car":
            rospy.Subscriber("/camera/image_raw", Image, self.horizon_image_callback)                           # Car horizontal RGB
            rospy.Subscriber("/depth_camera/depth/image_raw", Image, self.horizon_depth_callback)               # Car horizontal depth
            rospy.Subscriber("/odom", Odometry, self.odom_callback)                                             # Car pose
        elif self.device == "dog":
            rospy.Subscriber("/camera_face/color/image_raw", Image, self.horizon_image_callback)               # Robot dog horizontal RGB
            rospy.Subscriber("/camera_face/depth/image_raw", Image, self.horizon_depth_callback)               # Robot dog horizontal depth
            rospy.Subscriber("/gazebo/model_states", ModelStates, self.odom_callback)                         # Robot dog pose

        rospy.Subscriber("/agent_node/transform_command", String, self.command_callback)                    # Command from agent node

        self.env_desc_pub = rospy.Publisher('/vision_node/env_description', String, queue_size=10)          # Publish environment description text

    def birdeye_image_callback(self, msg):
        """Bird-eye RGB"""
        self.birdeye_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    def horizon_image_callback(self, msg):
        '''Horizontal RGB, object detection using World model'''
        self.horizon_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        cv_image = self.horizon_image
        results = self.model.predict(cv_image, verbose=False)       # Prediction, single image → only one result
        for i, result in enumerate(results):                        # Process each box
            self.result_boxes = boxes = result.boxes
            if i != 0:
                annotated_frame = annotated_frame.plot()            # Draw boxes
            else:
                annotated_frame = result.plot()

            for box in boxes:                                       # Get depth and draw
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                depth_value = self.get_depth_value(x1, y1, x2, y2)  # Get depth
                cv2.putText(annotated_frame, f"{depth_value:.2f}m", 
                            (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 4) 

        cv2.namedWindow("yolov8l-worldv2 Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("yolov8l-worldv2 Detection", 640, 480)
        cv2.imshow("yolov8l-worldv2 Detection", annotated_frame)
        cv2.waitKey(1)  # Wait 1ms for window update

    def horizon_image_callback_flask(self, msg):
        '''Horizontal RGB, vision model deployed on Flask backend'''
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

        # Convert image to base64
        _, buffer = cv2.imencode('.jpg', cv_image)
        img_base64 = base64.b64encode(buffer).decode('utf-8')

        # Send to Flask detection server
        payload = {'image': img_base64}
        response = self.send_to_flask(payload)
        if response:
            results = response.get('objects', [])
            self.result_boxes = results
            for obj in results:
                x1, y1, x2, y2 = obj['box']
                class_name = obj['label']
                confidence = obj['confidence']

                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                cv2.putText(cv_image, f"{class_name} {confidence:.2f}", 
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
                
                depth_value = self.get_depth_value(x1, y1, x2, y2)  
                cv2.putText(cv_image, f"{depth_value:.2f}m", 
                            (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 4) 

        cv2.namedWindow("yolov8l-worldv2 Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("yolov8l-worldv2 Detection", 640, 480)
        cv2.imshow("yolov8l-worldv2 Detection", cv_image)
        cv2.waitKey(1)

    def send_to_flask(self, payload):
        '''Send POST request to Flask server'''
        try:
            response = requests.post("http://localhost:5001/detect", json=payload)
            if response.status_code == 200:
                return response.json()
            else:
                rospy.logerr(f"Flask server returned error: {response.status_code}")
                return None
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Error sending request to Flask server: {e}")
            return None

    def horizon_depth_callback(self, msg):
        """Horizontal depth"""
        self.horizon_depth = CvBridge().imgmsg_to_cv2(msg, "passthrough")

    def odom_callback(self, msg):
        """Pose"""
        if isinstance(msg, Odometry):
            self.drone_position = msg.pose.pose.position
            self.drone_orientation = msg.pose.pose.orientation
        elif isinstance(msg, ModelStates):
            if "go1_gazebo" in msg.name:
                idx = msg.name.index("go1_gazebo")
                pose = msg.pose[idx]
                self.drone_position = pose.position
                self.drone_orientation = pose.orientation

    def command_callback(self, msg):
        """Command from agent node, generate environment description"""
        if msg.data == "get":
            self.get_world_coordinates()
        elif msg.data == "vlm":
            self.vlm(msg.data)

    def vlm(self, text):
        """Use VLM model to describe image"""
        cv_image = self.horizon_image
        cv2.imwrite("./run/monocular.jpg", cv_image)
        with open("./run/monocular.jpg", "rb") as image_file:
            base64_image = base64.b64encode(image_file.read()).decode("utf-8")
        client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY"),
            base_url=os.getenv("OPENAI_BASE_URL"),
        )
        completion = client.chat.completions.create(
            model="qwen-vl-plus",  # Model list
            messages=[{"role": "user","content": [
                    {"type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}},
                    {"type": "text", "text": f"Describe the environment in the simplest possible English (color/objects/facilities/buildings/landscape etc.). Less than 100 words."},
                    ]}]
        )
        self.env_desc_pub.publish(completion.choices[0].message.content)

    def get_world_coordinates(self):
        """Convert each detected object's pixel and depth to world coordinates, publish JSON"""
        boxes = self.result_boxes
        results = []  # Object info list

        for box in boxes:
            x1, y1, x2, y2 = map(int, box['box']) if isinstance(box, dict) else map(int, box.xyxy[0])
            depth_value = self.get_depth_value(x1, y1, x2, y2)
            coordinates = self.get_coordinates((x1 + x2) // 2, (y1 + y2) // 2, depth_value)

            if coordinates:
                obj_name = box['label'] if isinstance(box, dict) else self.model.names[int(box.cls)]
                results.append({
                    "name": obj_name,
                    "world_coordinates": {
                        "x": round(coordinates[0], 3),
                        "y": round(coordinates[1], 3),
                        "z": round(coordinates[2], 3),
                    },
                    "depth": round(float(depth_value), 3)
                })

        if not results:
            payload = {"status": "no_objects_detected"}
        else:
            payload = {"status": "ok", "objects": results}

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.env_desc_pub.publish(msg)

        print(f"📡 Published environment description: {msg.data}")  

    def get_depth_value(self, x1, y1, x2, y2):
        """Get depth value of the center of bounding box"""
        if self.horizon_depth is None:
            return float('nan')
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        depth_value = self.horizon_depth[center_y, center_x]
        depth_value = depth_value / 1000.0 if device_info[self.device]["depth"] == "mm" else depth_value

        if not np.isnan(depth_value) and 0 < depth_value < 100.0:
            return depth_value
        
        roi = self.horizon_depth[y1:y2, x1:x2]
        roi_valid = roi[(roi > 0) & (roi < 100.0) & (~np.isnan(roi))]

        if roi_valid.size > 0:
            min_depth = np.min(roi_valid)
            return min_depth / 1000.0 if device_info[self.device]["depth"] == "mm" else min_depth

        return float('nan')
    
    def get_coordinates(self, u, v, depth):
        """Convert pixel + depth to world coordinates"""
        camera_link = device_info[self.device]["camera_link"]
        base_link = device_info[self.device]["base_link"]
        camera_info = rospy.wait_for_message(device_info[self.device]["camerainfo_topic"], CameraInfo, timeout=5.0)
        camera_fx = camera_info.K[0]
        camera_fy = camera_info.K[4]
        camera_cx = camera_info.K[2]
        camera_cy = camera_info.K[5]

        camera_point = PointStamped()
        camera_point.header.frame_id = camera_link
        camera_point.point.y = - (u - camera_cx) * depth / camera_fx
        camera_point.point.z = - (v - camera_cy) * depth / camera_fy
        camera_point.point.x = float(depth)
        try:
            listener = tf.TransformListener()
            listener.waitForTransform(base_link, camera_link, rospy.Time(0), rospy.Duration(1.0))
            camera_point_in_base_link = listener.transformPoint(base_link, camera_point)

            if self.drone_position is None or self.drone_orientation is None:
                rospy.logerr("Pose not received!")
                return None
            
            qx, qy, qz, qw = self.drone_orientation.x, self.drone_orientation.y, self.drone_orientation.z, self.drone_orientation.w
            roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

            drone_position_map = [self.drone_position.x, self.drone_position.y, self.drone_position.z]

            rotation_matrix = self._get_rotation_matrix(yaw)

            camera_coords_in_base_link = [camera_point_in_base_link.point.x, camera_point_in_base_link.point.y, camera_point_in_base_link.point.z]
            camera_coords_in_map = self._apply_rotation_and_translation(camera_coords_in_base_link, rotation_matrix, drone_position_map)

            return tuple(round(x, 3) for x in camera_coords_in_map)

        except (tf.Exception) as e:
            rospy.logerr(f"Transform failed: {e}")
            return None
    
    def _get_rotation_matrix(self, yaw):
        """Compute rotation matrix from yaw angle"""
        rotation_matrix = [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ]
        return rotation_matrix

    def _apply_rotation_and_translation(self, camera_coords, rotation_matrix, translation_vector):
        """Apply rotation + translation to map coordinates"""
        rotated_coords = np.dot(rotation_matrix, camera_coords)
        transformed_coords = rotated_coords + np.array(translation_vector)
        return transformed_coords
    

if __name__ == '__main__':
    try:
        node = VisionNode()
        rospy.spin()   # Keep node running
    except rospy.ROSInterruptException:
        pass
