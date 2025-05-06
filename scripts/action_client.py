#!/usr/bin/env python

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal, RobotState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class RobotActionClient:
    def __init__(self):
        # Initialize the action client
        self.ac = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.ac.wait_for_server()
        
        # Publisher for robot state
        self.pub = rospy.Publisher('/robot_state', RobotState, queue_size=10)
        
        # Subscriber to odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Initialize robot state message
        self.robot_state = RobotState()

    def send_goal(self, x, y):
        # Create and send a new goal
        goal = PlanningGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # Neutral orientation
        
        # Send the goal with callbacks
        self.ac.send_goal(goal, 
                          done_cb=self.done_callback,
                          active_cb=self.active_callback,
                          feedback_cb=self.feedback_callback)
        rospy.loginfo("Goal sent to position (x: %f, y: %f)", x, y)

    def cancel_goal(self):
        # Cancel the current goal
        self.ac.cancel_goal()
        rospy.loginfo("Goal canceled.")

    def done_callback(self, status, result):
        # Called when the goal is finished
        rospy.loginfo("Goal completed with status: %s" % str(status))

    def active_callback(self):
        # Called when the goal becomes active
        rospy.loginfo("Goal is now active.")

    def feedback_callback(self, feedback):
        # Feedback callback (no action needed as per original code)
        pass

    def odom_callback(self, msg):
        # Update and publish robot state based on odometry
        self.robot_state.x = msg.pose.pose.position.x
        self.robot_state.y = msg.pose.pose.position.y
        self.robot_state.vel_x = msg.twist.twist.linear.x
        self.robot_state.vel_z = msg.twist.twist.angular.z
        self.pub.publish(self.robot_state)

def main():
    # Initialize the node
    rospy.init_node('robot_action_client')
    client = RobotActionClient()

    # User interaction loop
    while not rospy.is_shutdown():
        print("\n=============================================")
        print("Enter command:")
        print("s: Set a new goal")
        print("c: Cancel current goal")
        print("e: Exit program")
        command = input("Command: ").strip().lower()

        if command == 's':
            try:
                x = float(input("Enter target x: "))
                y = float(input("Enter target y: "))
                client.send_goal(x, y)
            except ValueError:
                print("Invalid input! Please enter numeric values.")
        elif command == 'c':
            client.cancel_goal()
        elif command == 'e':
            rospy.signal_shutdown("User requested exit.")
            break
        else:
            print("Invalid command. Please try again.")

if __name__ == '__main__':
    main()