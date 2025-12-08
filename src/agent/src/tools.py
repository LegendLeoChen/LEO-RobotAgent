import numpy as np
import os, json
import time

from langchain.agents import Tool
from langchain_openai import ChatOpenAI

import rospy
import tf
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion

from cv_bridge import CvBridge, CvBridgeError

class AgentTools:
    def __init__(self):
        self.uav_ctrl_pub = rospy.Publisher('/agent_node/uav_target_position', Pose, queue_size=10)     # Topic to control UAV target position
        self.car_ctrl_pub = rospy.Publisher('/agent_node/car_target_position', Pose, queue_size=10)     # Topic to control car target position
        self.arm_ctrl_pub = rospy.Publisher('/agent_node/grasp', String, queue_size=10)                 # Topic to control robotic arm grasping
        self.dog_ctrl_pub = rospy.Publisher('/agent_node/dog_target_position', Pose, queue_size=10)     # Topic to control robot dog target position
        self.vision_pub = rospy.Publisher('/agent_node/transform_command', String, queue_size=10)       # Send commands to the vision node

        self.build_tools()
        self.llm = ChatOpenAI(
            model="qwen3-max", 
            temperature=0.7,
        )

        self.path_record = []
        
    # ======================
    # Tool definitions
    # ======================
    def uav_fly(self, coor):
        """Publish target coordinates + yaw to the UAV control node.
        coor = {'x': float, 'y': float, 'z': float, 'yaw': float(degree)}
        """
        pose_msg = Pose()
        pose_msg.position.x = coor['x']
        pose_msg.position.y = coor['y']
        pose_msg.position.z = coor['z']
        pose_msg.orientation.z = coor['yaw'] if coor['yaw'] is not None else 0.0  # Only use z field to send yaw (degree)

        self.path_record.append([coor['x'], coor['y'], coor['z'], coor['yaw']])
        self.uav_ctrl_pub.publish(pose_msg)
        try:
            msg = rospy.wait_for_message('/fly_control_node/uav_reached', Bool, timeout=30.0)       # Wait until UAV reaches the target
            return msg.data
        except rospy.ROSException:
            return False
    
    def car_run(self, coor):
        """Publish target coordinates + yaw to the car control node.
        coor = {'x': float, 'y': float, 'yaw': float(degree)}
        """
        pose_msg = Pose()
        pose_msg.position.x = coor['x']
        pose_msg.position.y = coor['y']
        pose_msg.orientation.z = coor['yaw'] if coor['yaw'] is not None else 0.0

        self.car_ctrl_pub.publish(pose_msg)
        try:
            msg = rospy.wait_for_message('/car_control_node/car_reached', Bool, timeout=30.0)       # Wait until car reaches the target
            return msg.data
        except rospy.ROSException:
            return False
    
    def dog_run(self, coor):
        """Publish target coordinates to the robot dog control node.
        coor = {'x': float, 'y': float}
        """
        pose_msg = Pose()
        pose_msg.position.x = coor['x']
        pose_msg.position.y = coor['y']

        self.dog_ctrl_pub.publish(pose_msg)
        try:
            msg = rospy.wait_for_message('/dog_control_node/dog_reached', Bool, timeout=30.0)       # Wait until robot dog reaches the target
            return msg.data
        except rospy.ROSException:
            return False
    
    def add(self, nums):
        return nums['a'] + nums['b']

    def arm_grasp(self, parse="grasp"):
        """Grasp or release the object"""
        self.arm_ctrl_pub.publish(parse)
        try:
            msg = rospy.wait_for_message('/arm_control_node/grasp_finish', Bool, timeout=20.0)       # Wait for the grasping action to finish
            return msg.data
        except rospy.ROSException:
            return False
    
    def vlm(self, text=None):
        """Use VLM to generate image description"""
        self.vision_pub.publish("vlm")
        try:
            desc_text = rospy.wait_for_message('/vision_node/env_description', String, timeout=20.0)       # Receive environment description
        except:
            return "No valid objects detected in the current frame."
        return desc_text.data
    
    def object_detection(self, text=None):
        """World-model object detection"""
        self.vision_pub.publish("get")                          # Command vision node to generate environment JSON
        now = time.time()
        try:
            desc_text = rospy.wait_for_message('/vision_node/env_description', String, timeout=10.0)
        except:
            return "No objects in the current image."
        print('Detection time:', time.time() - now)

        response = self.llm.invoke(f"""example：A person(4.952, 0.623, 0.757) and a cat(1.898, 4.586, 0.222) is near a TV(6.907, 2.809, 2.226).
                                   Refer to the example, speak only English the whole time, no useless talk, no fabrication. Use only the simplest plain English to describe the environment formed by this text. For each target, add its coordinates in parentheses (nothing else). The data is as follows: {desc_text.data}""")
        return response.content if hasattr(response, "content") else response
    
    def object_detection_json(self, text=None):
        """World-model object detection"""
        self.vision_pub.publish("get")
        now = time.time()
        try:
            desc_text = rospy.wait_for_message('/vision_node/env_description', String, timeout=10.0)
        except:
            return {"status": "no_objects_detected", "objects": []}
        print('Detection time:', time.time() - now)
        data = json.loads(desc_text.data)
        return data
    
    def speak(self, text):
        return True

    def listen(self, parse):
        """Listen to environmental voice"""
        origin_text = "Hello, please return to the origin and continue your search. Help me find a bottle and grab it (yaw = 0°, for the car the grasping location is (target_x - 0.5, target_y)), then deliver it to the person near the lamp (stop at that person's x, y - 1)."
        response = self.llm.invoke(f"""You are a robot operator. Here is a command spoken by a human. Please rewrite it into several step-by-step instructions (1. …; 2. …), in less than 100 words:
                                   {origin_text}""")
        return f"Original: {origin_text}\nPlan Summary: {response.content if hasattr(response, 'content') else response}"

    def build_tools(self, json_path="./src/agent/config/tools.json"):
        """Load tool definitions from JSON and dynamically bind functions"""
        self.tools = []

        if not os.path.exists(json_path):
            raise FileNotFoundError(f"❌ Tool config file not found: {json_path}")

        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        for t in data.get("tools", []):
            if not t.get("activation", False):  # Skip disabled tools
                continue

            func_name = t.get("func")
            func = getattr(self, func_name, None)
            if func is None:
                print(f"⚠️ Warning: Function {func_name} not found, skip tool {t.get('name')}")
                continue

            tool = Tool(
                name=t.get("name", "unknown"),
                func=func,
                description=t.get("description", "")
            )
            self.tools.append(tool)

        print(f"✅ Loaded {len(self.tools)} active tools")
    
    def get_tools_description(self):
        """Get tool descriptions"""
        return "\n".join([f"{i+1}. {tool.name}: {tool.description};" for i, tool in enumerate(self.tools)])
