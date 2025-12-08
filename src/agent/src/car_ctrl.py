#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import tf


class CarControlNode:
    def __init__(self):
        rospy.init_node("car_control_node")
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/agent_node/car_target_position", Pose, self.goal_callback)                   # Target position sent by agent node
        self.reach_pub = rospy.Publisher('/car_control_node/car_reached', Bool, queue_size=10)          # Feedback to agent when goal reached

        self.pos = (0.0, 0.0)
        self.yaw = 0.0
        self.goal_pos = (0.0, 0.0)
        self.goal_yaw = 0.0

        # control parameters
        self.k_rho = 0.8
        self.k_alpha = 1.5
        self.k_beta = -0.4          # Must be negative to converge

        # thresholds
        self.pos_tolerance = 0.1
        self.yaw_tolerance = 0.05

        # state flag
        self.goal_reached = False

    def odom_callback(self, msg):
        """Current position of the car"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pos = (x, y)
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = yaw

    def goal_callback(self, msg):
        """Target position for the car"""
        self.goal_pos = (msg.position.x, msg.position.y)
        self.goal_yaw = math.radians(msg.orientation.z)
        rospy.loginfo(f"Target updated: ({msg.position.x}, {msg.position.y})m, yaw={self.goal_yaw}°")
        self.goal_reached = False                           # Reset state

    def compute_control(self):
        """Feedback linearization control"""
        cmd = Twist()

        dx = self.goal_pos[0] - self.pos[0]
        dy = self.goal_pos[1] - self.pos[1]
        rho = math.hypot(dx, dy)

        if rho > self.pos_tolerance:
            # desired direction
            goal_theta = math.atan2(dy, dx)
            alpha = math.atan2(math.sin(goal_theta - self.yaw),
                               math.cos(goal_theta - self.yaw))
            beta = math.atan2(math.sin(self.goal_yaw - goal_theta),
                              math.cos(self.goal_yaw - goal_theta))

            # control law
            v = self.k_rho * rho
            w = self.k_alpha * alpha + self.k_beta * beta

            cmd.linear.x = min(v, 0.6)  # speed limit
            cmd.angular.z = max(-1.0, min(1.0, w))  # limit angular velocity
        else:
            # adjust final orientation
            yaw_error = math.atan2(math.sin(self.goal_yaw - self.yaw),
                                   math.cos(self.goal_yaw - self.yaw))
            if abs(yaw_error) > self.yaw_tolerance:
                cmd.angular.z = 0.5 * yaw_error
            else:
                if not self.goal_reached:           # ✅ publish only once
                    rospy.loginfo("✅ Car has reached the target!")
                    self.reach_pub.publish(Bool(data=True))
                    self.goal_reached = True
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        return cmd

    def run(self):
        """Continuously control to the target"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            cmd = self.compute_control()
            self.cmd_pub.publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    CarControlNode().run()
