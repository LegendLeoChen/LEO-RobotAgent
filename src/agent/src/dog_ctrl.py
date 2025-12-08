#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import math

class DogControlNode:
    def __init__(self):
        rospy.init_node("dog_control_node", anonymous=True)
        self.pub = rospy.Publisher("/dog_cmd", String, queue_size=10)               # Send control command to Go1
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.state_callback)  # Subscribe to Go1 position state
        rospy.Subscriber("/agent_node/dog_target_position", Pose, self.go_to_goal)  # Target position from agent node
        self.reach_pub = rospy.Publisher('/dog_control_node/dog_reached', Bool, queue_size=10)  # Feedback to agent when goal reached

        self.current_x = 0.0
        self.current_y = 0.0
        self.have_state = False

        rospy.sleep(0.5)
        self.start()
    
    def state_callback(self, msg: ModelStates):
        """Current state of the robot dog"""
        if "go1_gazebo" in msg.name:
            idx = msg.name.index("go1_gazebo")
            pose = msg.pose[idx]
            self.current_x = pose.position.x
            self.current_y = pose.position.y
            self.have_state = True

    def send_cmd(self, cmd_str):
        """Send control command to Go1"""
        msg = String()
        msg.data = cmd_str
        self.pub.publish(msg)
        # rospy.loginfo("Published: %s", msg.data)

    def start(self):
        """Make the dog stand and enter walking mode"""
        self.send_cmd("CMD:2")
        rospy.sleep(0.02)
        self.send_cmd("CMD:4")

    def go_to_goal(self, msg: Pose):
        """Receive target from agent and move to that position"""
        self.run_to(msg.position.x, msg.position.y)

    def run_to(self, target_x, target_y, velocity=3.0, tol=0.05):
        """Closed-loop walking to target position (x, y), stop after arrival"""
        rospy.loginfo(f"Current: {self.current_x:.2f}, {self.current_y:.2f}, moving to: {target_x:.2f}, {target_y:.2f}")
        rate = rospy.Rate(10)  # 10Hz control

        # Wait for Gazebo state
        while not self.have_state and not rospy.is_shutdown():
            rospy.logwarn("Waiting for gazebo state...")
            rate.sleep()

        while not rospy.is_shutdown():
            # compute direction vector
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < tol:
                self.send_cmd("CMD: ")                          # Stop after reaching
                rospy.loginfo("✅ Dog has reached the target!")
                self.reach_pub.publish(Bool(data=True))         # Feedback to agent
                break

            # normalize direction
            lx = dx / dist
            ly = dy / dist

            # convert to percentage
            lx_percent = int(lx * 100 * velocity)
            ly_percent = int(ly * 100 * velocity)

            # construct direction command (only one axis per WS/AD)
            cmd_parts = []

            # x-direction
            if lx_percent > 0:
                cmd_parts.append(f"W={lx_percent}")
            elif lx_percent < 0:
                cmd_parts.append(f"S={-lx_percent}")  # positive number

            # y-direction
            if ly_percent > 0:
                cmd_parts.append(f"A={ly_percent}")
            elif ly_percent < 0:
                cmd_parts.append(f"D={-ly_percent}")

            cmd = "DIR:" + ",".join(cmd_parts)
            self.send_cmd(cmd)
            rospy.logdebug("Sent direction command: %s", cmd)

            rate.sleep()
        
    def test(self):
        """Test control commands with Go1"""
        rospy.sleep(1)        # wait to ensure subscriber connected
        self.send_cmd("CMD:2")
        rospy.sleep(1)
        self.send_cmd("CMD:4")
        rospy.sleep(1)
        self.send_cmd("VEL:0.8")
        rospy.sleep(2.5)
        self.send_cmd("DIR:W=55,A=45")
        rospy.sleep(5.5)
        self.send_cmd("CMD: ")


if __name__ == "__main__":
    commander = DogControlNode()
    rospy.sleep(1)

    # commander.run_to(0.0, 0.0, velocity=3.0)
    rospy.spin()
