#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import moveit_commander
from std_msgs.msg import String, Bool
from tf.transformations import quaternion_from_euler


class ArmControlNode(object):
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        rospy.init_node('arm_control_node', anonymous=True)

        self.arm_group = moveit_commander.MoveGroupCommander("arm")             # Robot arm, same as MoveIt! group name
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")     # Gripper

        rospy.Subscriber('/agent_node/grasp', String, self.arm_grasp)           # Grasp command from agent node
        self.finish_pub = rospy.Publisher('/arm_control_node/grasp_finish', Bool, queue_size=10)    # Feedback to agent node: grasp finished

        # Move to a default pose at initialization
        rospy.sleep(1)
        # self.arm_grasp()    # Do not remove commented code

    def move_to(self, x, y, z, roll, pitch, yaw):
        """Use MoveIt planning to move the gripper palm (link6) to the target pose"""
        q = quaternion_from_euler(roll, pitch, yaw)

        self.arm_group.set_planning_time(3)
        default_pose = PoseStamped()
        default_pose.header.frame_id = "base_link"
        default_pose.pose.position.x = x
        default_pose.pose.position.y = y
        default_pose.pose.position.z = z
        default_pose.pose.orientation.x = q[0]
        default_pose.pose.orientation.y = q[1]
        default_pose.pose.orientation.z = q[2]
        default_pose.pose.orientation.w = q[3]

        self.arm_group.set_pose_target(default_pose)
        success = self.arm_group.go(wait=True)
        # print(success)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        if success:
            rospy.loginfo("Move completed successfully.")
        else:
            rospy.logwarn("Move failed!")

    def set_gripper(self, angle):
        """Set the angle of gripper link7/link8, angle in radians"""
        joint_goal = self.gripper_group.get_current_joint_values()
        joint_goal[0] = angle
        joint_goal[1] = -angle  # If your gripper is symmetric
        self.gripper_group.go(joint_goal, wait=True)
        self.gripper_group.stop()
        # rospy.loginfo(f"Gripper set to {angle} rad")    # Keep commented code
    
    def arm_grasp(self, msg=None):
        """Grasp and release actions"""
        if msg.data == "grasp":         # Grasp
            self.set_gripper(0.2)                                   # Open gripper
            self.move_to(0.56, 0.0, 0.327, 0.035, 1.54, 0.04)       # Move forward and down to grab
            rospy.sleep(2)
            self.set_gripper(-0.2)                                  # Close gripper
            rospy.sleep(3)
            self.move_to(0.0, 0.0, 0.84, 0.0, 0.0, 0.0)             # Lift up
        else:                           # Release
            self.move_to(0.56, 0.0, 0.327, 0.035, 1.54, 0.04)
            rospy.sleep(2)
            self.set_gripper(0.2)
            rospy.sleep(2)
            self.move_to(0.0, 0.0, 0.84, 0.0, 0.0, 0.0)
        self.finish_pub.publish(Bool(data=True))
        rospy.loginfo("✅ Grasp action completed!")

if __name__ == "__main__":
    mover = ArmControlNode()
    rospy.spin()
