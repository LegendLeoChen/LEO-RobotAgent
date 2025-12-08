#!/usr/bin/env python3
import rospy
import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, SetModelState
from gazebo_msgs.msg import ModelState, ModelStates

class FollowCameraNode:
    def __init__(self):
        rospy.init_node("follow_camera_node")

        # --- Camera Parameters ---
        self.camera_name = "follow_camera"
        self.relative_offset = np.array([-7.0, -7.0, 5.0])  # x, y, z relative to base_link
        self.pitch = 30.0  # Camera pitch angle
        self.yaw = 45.0    # Camera yaw angle
        self.rate_hz = 20

        self.target_pose = None  # Store base_link pose
        self.found = False

        # --- Spawn camera model ---
        self.spawn_camera()

        # --- Subscribe to Gazebo model state ---
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

        # --- Gazebo set_model_state service ---
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # --- Start loop ---
        self.run()

    def spawn_camera(self):
        """Dynamically generate an SDF camera model and spawn it in Gazebo"""
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        camera_sdf = f"""
        <sdf version="1.6">
          <model name="{self.camera_name}">
            <static>true</static>
            <link name="follow_camera_link">
              <sensor name="camera_sensor" type="camera">
                <always_on>true</always_on>
                <update_rate>30.0</update_rate>
                <camera>
                  <horizontal_fov>1.047</horizontal_fov>
                  <image>
                    <width>1280</width>
                    <height>960</height>
                    <format>R8G8B8</format>
                  </image>
                  <clip>
                    <near>0.1</near>
                    <far>100.0</far>
                  </clip>
                </camera>
                <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
                  <always_on>true</always_on>
                  <update_rate>30.0</update_rate>
                  <camera_name>{self.camera_name}</camera_name>
                  <image_topic_name>image_raw</image_topic_name>
                  <camera_info_topic_name>camera_info</camera_info_topic_name>
                  <frame_name>follow_camera_link</frame_name>
                  <robotNamespace>/{self.camera_name}</robotNamespace>
                </plugin>
              </sensor>
            </link>
          </model>
        </sdf>
        """

        initial_pose = Pose()
        initial_pose.position.x = 0
        initial_pose.position.y = 0
        initial_pose.position.z = 1.0

        try:
            spawn_model(self.camera_name, camera_sdf, "", initial_pose, "world")
            rospy.loginfo(f"📷 Camera '{self.camera_name}' spawned in Gazebo.")
        except Exception as e:
            rospy.logerr(f"Failed to spawn camera: {e}")

    def model_states_callback(self, msg):
        """Retrieve target base_link pose from Gazebo"""
        target_names = ["robot", "iris", "go1_gazebo"]  # Allowed model names

        try:
            # Find the first existing target model
            for name in target_names:
                if name in msg.name:
                    index = msg.name.index(name)
                    self.target_pose = msg.pose[index]
                    if not self.found:
                        rospy.loginfo(f"Target model found: {name}")
                        self.found = True
                    return

            rospy.logwarn_throttle(5, f"None of the target models found: {target_names}")

        except Exception as e:
            rospy.logerr_throttle(5, f"Error in model_states_callback: {e}")

    def compute_camera_pose(self):
        """Compute camera pose using fixed offset + fixed orientation"""
        if self.target_pose is None:
            return None

        cam_pose = Pose()
        cam_pose.position.x = self.target_pose.position.x + self.relative_offset[0]
        cam_pose.position.y = self.target_pose.position.y + self.relative_offset[1]
        cam_pose.position.z = self.target_pose.position.z + self.relative_offset[2]

        # Fixed pitch and yaw
        roll = 0
        pitch = np.radians(self.pitch)
        yaw_cam = np.radians(self.yaw)
        q_cam = tft.quaternion_from_euler(roll, pitch, yaw_cam)
        cam_pose.orientation.x = q_cam[0]
        cam_pose.orientation.y = q_cam[1]
        cam_pose.orientation.z = q_cam[2]
        cam_pose.orientation.w = q_cam[3]

        return cam_pose

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            cam_pose = self.compute_camera_pose()
            if cam_pose is not None:
                state_msg = ModelState()
                state_msg.model_name = self.camera_name
                state_msg.pose = cam_pose
                state_msg.reference_frame = "world"
                try:
                    self.set_model_state_srv(state_msg)
                except Exception as e:
                    rospy.logwarn(f"Failed to update camera pose: {e}")
            rate.sleep()


if __name__ == "__main__":
    FollowCameraNode()
