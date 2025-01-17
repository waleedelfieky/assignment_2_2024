#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2024/PlanningAction.h>
#include <nav_msgs/Odometry.h>
#include <assignment_2_2024/RobotState.h>
#include <iostream>

typedef actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> PlanningClient;

class RobotActionClient
{
public:
    RobotActionClient() : ac("reaching_goal", true)
    {
        ac.waitForServer();
        pub = nh.advertise<assignment_2_2024::RobotState>("robot_state", 10);
        odom_sub = nh.subscribe("/odom", 10, &RobotActionClient::odomCallback, this);
    }

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

    void cancelGoal()
    {
        ac.cancelGoal();
        ROS_INFO("Goal canceled.");
    }

private:
    ros::NodeHandle nh;
    PlanningClient ac;
    ros::Publisher pub;
    ros::Subscriber odom_sub;
    assignment_2_2024::RobotState robot_state;

    void doneCb(const actionlib::SimpleClientGoalState &state, const assignment_2_2024::PlanningResultConstPtr &result)
    {
        ROS_INFO("Goal finished with state: %s", state.toString().c_str());
    }

    void activeCb()
    {
        ROS_INFO("Goal just went active");
    }

    void feedbackCb(const assignment_2_2024::PlanningFeedbackConstPtr &feedback)
    {
        // Placeholder for feedback processing if needed
        //ROS_INFO("Got Feedback");

    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        robot_state.x = msg->pose.pose.position.x;
        robot_state.y = msg->pose.pose.position.y;
        robot_state.vel_x = msg->twist.twist.linear.x;
        robot_state.vel_z = msg->twist.twist.angular.z;
        pub.publish(robot_state);
    }
};

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

