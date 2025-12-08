#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import quaternion_from_euler
from pynput import keyboard


class FlyControlNode:
    def __init__(self):
        self.target_pos = PoseStamped()
        self.target_yaw = 0.0
        self.current_pose = None
        self.step = 1.0                             # Movement step (meters)
        self.yaw_step = math.radians(10)            # Yaw rotation step (radians)
        self.control_enabled = False                # F1 toggles manual keyboard control
        self.reach_tolerance = 0.2                  # Position tolerance (meters)
        self.yaw_tolerance = 5.0                    # Yaw tolerance (degrees)

        rospy.init_node('fly_control_node')

        rospy.Subscriber('/agent_node/uav_target_position', Pose, self.target_callback)  # Target from Agent Node
        self.goal_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)  
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.reach_pub = rospy.Publisher('/fly_control_node/uav_reached', Bool, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_target)
        # self.set_target_position(-60, 0, 10, 0)        # Default hover at (0,0,3), yaw=0°
        self.set_target_position(0, 0, 3, 0)

        rospy.sleep(3)
        self.arm()
        rospy.sleep(1)
        self.set_mode("OFFBOARD")

        # Start keyboard listener
        listener = keyboard.Listener(on_press=self.on_key_press)
        listener.daemon = True
        listener.start()

    def set_mode(self, mode):
        """Set UAV flight mode through MAVROS (e.g., OFFBOARD)"""
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            set_mode_srv(0, mode)
        except rospy.ServiceException as e:
            rospy.logerr(f"Set mode failed: {e}")

    def arm(self):
        """Arm the UAV"""
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_srv(True)
        except rospy.ServiceException as e:
            rospy.logerr(f"Arming failed: {e}")

    def pose_callback(self, msg):
        """Receive current UAV pose from MAVROS"""
        self.current_pose = msg.pose

    def set_target_position(self, x, y, z, yaw_deg):
        """Set target position + yaw angle (degree input)"""
        self.target_pos.pose.position.x = x
        self.target_pos.pose.position.y = y
        self.target_pos.pose.position.z = z
        self.target_yaw = math.radians(yaw_deg)
        rospy.loginfo(f"Target updated: ({x}, {y}, {z})m, yaw={yaw_deg}°")

    def target_callback(self, msg):
        """Receive target from Agent node"""
        self.set_target_position(msg.position.x, msg.position.y, msg.position.z, msg.orientation.z)
        reached = self.check_reached(
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.z
        )
        self.reach_pub.publish(Bool(data=reached))

    def check_reached(self, target_x, target_y, target_z, target_yaw_deg, timeout_sec=20):
        """Check if UAV has reached the target position and yaw"""
        rate = rospy.Rate(5)
        timeout = rospy.Time.now() + rospy.Duration(timeout_sec)

        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.logwarn_throttle(5, "Waiting for UAV pose data...")
                rate.sleep()
                continue

            # --- Position error ---
            dx = self.current_pose.position.x - target_x
            dy = self.current_pose.position.y - target_y
            dz = self.current_pose.position.z - target_z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

            # --- Extract current yaw from quaternion ---
            q = self.current_pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
            current_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

            # --- Yaw error ---
            yaw_diff = abs((current_yaw - target_yaw_deg + 180) % 360 - 180)

            # --- Reach detection ---
            if dist < self.reach_tolerance and yaw_diff < self.yaw_tolerance:
                rospy.loginfo("✅ Target reached!")
                rospy.sleep(0.01)
                return True

            # --- Timeout ---
            if rospy.Time.now() > timeout:
                return False

            rate.sleep()

    def publish_target(self, event):
        """Publish target position + yaw to MAVROS"""
        self.target_pos.header.stamp = rospy.Time.now()
        self.target_pos.header.frame_id = "map"

        q = quaternion_from_euler(0, 0, self.target_yaw)
        self.target_pos.pose.orientation.x = q[0]
        self.target_pos.pose.orientation.y = q[1]
        self.target_pos.pose.orientation.z = q[2]
        self.target_pos.pose.orientation.w = q[3]

        self.goal_pub.publish(self.target_pos)

    def on_key_press(self, key):
        """Keyboard manual control handler"""
        try:
            # F1 key toggles manual control
            if hasattr(key, "vk") and key.vk == 269025027:
                self.control_enabled = not self.control_enabled
                state = "ENABLED" if self.control_enabled else "DISABLED"
                rospy.loginfo(f"Keyboard control {state}")
                return

            if not self.control_enabled:
                return

            # Arrow keys: X-Y translation
            if key == keyboard.Key.up:
                self.target_pos.pose.position.x += self.step
            elif key == keyboard.Key.down:
                self.target_pos.pose.position.x -= self.step
            elif key == keyboard.Key.left:
                self.target_pos.pose.position.y += self.step
            elif key == keyboard.Key.right:
                self.target_pos.pose.position.y -= self.step

            # WASD: Z movement and yaw rotation
            if hasattr(key, 'char'):
                if key.char == 'w':
                    self.target_pos.pose.position.z += self.step
                elif key.char == 's':
                    self.target_pos.pose.position.z -= self.step
                elif key.char == 'a':
                    self.target_yaw += self.yaw_step
                elif key.char == 'd':
                    self.target_yaw -= self.yaw_step
                elif key.char in ['o', 'O']:
                    self.set_target_position(0, 0, 3, 0)

        except Exception as e:
            rospy.logwarn(f"Key handling error: {e}")


if __name__ == '__main__':
    node = FlyControlNode()
    rospy.spin()
