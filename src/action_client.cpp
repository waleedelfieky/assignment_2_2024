/**
* \file action_client.cpp 
* \brief action client to control mobile robot movement
* \author Elfieky Waleed
* \version 0.1
* \date 24/03/2025
*
* \details
*
* Subscribes to: <BR>
*   /odom — receives the robot's odometry data.
*
* Publishes to: <BR>
*   /robot_state — publishes current position and velocity of the robot.
*
* Action Clients: <BR>
*   /reaching_goal — sends position goals using the PlanningAction interface.
*
* Description:
*
* A node that implements an action client, allowing the userto set a target (x, y) or to cancel it. Try to use the
* feedback/status of the action server to know when the target has been reached. The node also publishesthe
* robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the
* topic /odom;
**/


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2024/PlanningAction.h>
#include <nav_msgs/Odometry.h>
#include <assignment_2_2024/RobotState.h>
#include <iostream>

/**
 * \typedef PlanningClient
 * \brief Typedef for a SimpleActionClient using the PlanningAction defined in the assignment_2_2024 package.
 *
 * This action client is used to send goals to the 'reaching_goal' action server.
 */

typedef actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> PlanningClient;

/**
 * \class RobotActionClient
 * \brief A ROS action client that sends navigation goals to the robot and publishes its current state.
 *
 * This class connects to the 'reaching_goal' action server, allows sending and canceling goals,
 * listens to the robot's odometry, and publishes the robot's current state (position and velocity)
 * on the 'robot_state' topic.
 */

 class RobotActionClient
{
public:
    /**
     * @brief Constructor.
     *
     * Initializes the action client, waits for the server, sets up the odometry subscriber and the robot_state publisher.
     */
    RobotActionClient() : ac("reaching_goal", true)
    {
        ac.waitForServer();
        pub = nh.advertise<assignment_2_2024::RobotState>("robot_state", 10);
        odom_sub = nh.subscribe("/odom", 10, &RobotActionClient::odomCallback, this);
    }
    /**
     * \brief Sends a goal to the action server.
     * 
     * \param x X-coordinate of the target position.
     * \param y Y-coordinate of the target position.
     * 
     * \return it return void 
     * 
     * this function send a goal of x and y to the action server which then will move the mobile robot 
     * to this position with consideration of obsticales
     */

    void sendGoal(double x, double y)
    {
        assignment_2_2024::PlanningGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation.w = 1.0;

        ac.sendGoal(goal,
                    boost::bind(&RobotActionClient::doneCb, this, _1, _2),
                    boost::bind(&RobotActionClient::activeCb, this),
                    boost::bind(&RobotActionClient::feedbackCb, this, _1));
    }
    /**
     * \brief Cancels the current goal sent to the action server.
     * 
     * the function take no argument and return void it only cancell the goal using the action client varibale
     * that we defined before in the class
     */
    void cancelGoal()
    {
        ac.cancelGoal();
        ROS_INFO("Goal canceled.");
    }

private:
    /**
    * \brief ROS node handle used to initialize publishers and subscribers.
    */
    ros::NodeHandle nh;
    /**
    * \brief Action client used to send goals to the 'reaching_goal' action server.
    */
    PlanningClient ac;
    /**
    * \brief Publisher for the robot's current state (position and velocity).
    */
    ros::Publisher pub;
    /**
    * \brief Subscriber to the robot's odometry data.
    */
    ros::Subscriber odom_sub;
    /**
    * \brief Struct that stores the robot's current x, y position and velocities.
    */
    assignment_2_2024::RobotState robot_state;
   /**
    * \brief Callback function called when the goal is finished.
    *
    * \param state Final state of the goal.
    * \param result Result returned by the action server.
    */
    void doneCb(const actionlib::SimpleClientGoalState &state, const assignment_2_2024::PlanningResultConstPtr &result)
    {
        ROS_INFO("Goal finished with state: %s", state.toString().c_str());
    }
    /**
    * \brief Callback function called when the goal becomes active.
    */
    void activeCb()
    {
        ROS_INFO("Goal just went active");
    }
    /**
    * \brief Callback function called with feedback during goal execution.
    *
    * \param feedback Feedback data from the action server.
    */
    void feedbackCb(const assignment_2_2024::PlanningFeedbackConstPtr &feedback)
    {
        // Placeholder for feedback processing if needed
        //ROS_INFO("Got Feedback");

    }
    /**
    * \brief Callback function for odometry updates.
    *
    * Updates the robot_state and publishes it.
    * 
    * \param msg Pointer to the received odometry message.
    */
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        robot_state.x = msg->pose.pose.position.x;
        robot_state.y = msg->pose.pose.position.y;
        robot_state.vel_x = msg->twist.twist.linear.x;
        robot_state.vel_z = msg->twist.twist.angular.z;
        pub.publish(robot_state);
    }
};


/**
 * \brief Main function that initializes the ROS node and handles user interaction.
 *
 * Initializes the ROS node and an instance of RobotActionClient. 
 * It continuously prompts the user for commands to:
 * - Send a goal to the robot (command 's')
 * - Cancel the current goal (command 'c')
 * - Exit the program (command 'e')
 *
 * The robot will navigate to the target (x, y) position if a goal is sent.
 *
 * \param argc Argument count (unused).
 * \param argv Argument vector (unused).
 * \return int Exit status.
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_action_client");
    RobotActionClient client;

    char command;
    int x, y;

    while (ros::ok())
    {
    	std::cout<<"============================================="<<std::endl;
        std::cout<< "Please enter command"<<std::endl;
        std::cout<<"s: set"<<std::endl;
        std::cout<<"c: For Cancel"<<std::endl;
        std::cout<<"e: For End" << std::endl;
        std::cin >> command;
        

        if (command == 's')
        {
            std::cout<<"============================================="<<std::endl;
            std::cout << "Enter x: ";
            std::cin >> x;
            std::cout << "Enter y: ";
            std::cin >> y;
            std::cout << "Robot will go to x: " << x << " and y: " << y <<std::endl;
            std::cout<<"============================================="<<std::endl;
            client.sendGoal(x, y);
        }
        else if (command == 'c')
        {
            client.cancelGoal();
        }
        else if (command == 'e')
        {
            break;
        }
        else
        {
            std::cout << "You have entered invalid input." << std::endl;
        }

        ros::spinOnce(); // Process ROS callbacks without blocking
        ros::Duration(1).sleep(); // Small sleep to allow console output to refresh properly
    }

    return 0;
}

