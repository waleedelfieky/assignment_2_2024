#!/usr/bin/env python

import rospy
from assignment_2_2024.srv import GetTarget, GetTargetResponse
from assignment_2_2024.msg import PlanningActionGoal

# Global variables to store the target coordinates
target_x = 0.0
target_y = 0.0

def goal_callback(msg):
    """
    Callback function for the goal subscriber.
    Updates the global target coordinates when a new goal is received.
    """
    global target_x, target_y
    target_x = msg.goal.target_pose.pose.position.x
    target_y = msg.goal.target_pose.pose.position.y
    rospy.loginfo("Updated target position: x = %f, y = %f", target_x, target_y)

def handle_get_target(req):
    """
    Service callback that returns the current target coordinates.
    """
    response = GetTargetResponse()
    response.x = target_x
    response.y = target_y
    rospy.loginfo("Service called. Returning position: x = %f, y = %f", response.x, response.y)
    return response

def main():
    # Initialize the node
    rospy.init_node('get_target_service_node')
    
    # Create subscriber for goal updates
    rospy.Subscriber("/reaching_goal/goal", PlanningActionGoal, goal_callback)
    
    # Create service server
    service = rospy.Service("get_target", GetTarget, handle_get_target)
    
    rospy.loginfo("Service node is ready to provide target position.")
    
    # Keep the node running
    rospy.spin()

if __name__ == "__main__":
    main()