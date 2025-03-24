/**
* \file target_service.cpp 
* \brief Service node to provide target position of the robot.
* \author Elfieky Waleed
* \version 0.1
* \date 24/03/2025
*
* \details
*
* Subscribes to: <BR>
*   /reaching_goal/goal — receives the goal target position for the robot. <BR>
*
* Services: <BR>
*   /get_target — provides the latest target position (x, y) requested by the user. <BR>
*
* Description:
*
* A service node that, when called, returns the coordinates of the last target sent by the user. 
* It subscribes to the /reaching_goal/goal topic to update the target coordinates and offers a service 
* that allows clients to retrieve the latest target position.
**/

#include "ros/ros.h"
#include "assignment_2_2024/GetTarget.h" ///< Custom service header
#include "assignment_2_2024/PlanningActionGoal.h" ///< Message header for PlanningActionGoal


/**
 * \brief Stores the latest goal x-coordinate.
 */
double target_x = 0.0;

/**
 * \brief Stores the latest goal y-coordinate.
 */
double target_y = 0.0;

/**
 * \brief Callback function for the goal topic subscriber.
 *
 * Updates the global `target_x` and `target_y` variables whenever a new goal is published.
 *
 * \param msg Pointer to the received PlanningActionGoal message.
 */
void goalCallback(const assignment_2_2024::PlanningActionGoal::ConstPtr& msg) {
    target_x = msg->goal.target_pose.pose.position.x;
    target_y = msg->goal.target_pose.pose.position.y;

    ROS_INFO("Updated target position: x = %f, y = %f", target_x, target_y);
}

/**
 * \brief Service callback to return the current target position.
 *
 * Responds to service requests by returning the latest known target x and y coordinates.
 *
 * \param req Incoming service request (unused).
 * \param res Outgoing service response containing target x and y.
 * \return true Always returns true to indicate the service was handled.
 */

bool getTargetPosition(assignment_2_2024::GetTarget::Request &req,
                       assignment_2_2024::GetTarget::Response &res) {
    res.x = target_x;
    res.y = target_y;

    ROS_INFO("Service called. Returning position: x = %f, y = %f", res.x, res.y);
    return true;
}

/**
 * \brief Main function to initialize the service node.
 *
 * Subscribes to the `/reaching_goal/goal` topic to track the robot's target position.
 * Advertises a service called `/get_target` to return the current target position.
 *
 * \param argc Argument count.
 * \param argv Argument vector.
 * \return int Exit code.
 */

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "get_target_service_node");
    ros::NodeHandle nh;

    // Subscriber to the /reaching_goal/goal topic
    ros::Subscriber goal_sub = nh.subscribe("/reaching_goal/goal", 10, goalCallback);

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("get_target", getTargetPosition);

    ROS_INFO("Service node is ready to provide target position.");
    ros::spin();

    return 0;
}

