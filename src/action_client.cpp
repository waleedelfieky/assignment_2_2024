#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <assignment_2_2024/RobotState.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RobotActionClient
{
public:
    RobotActionClient() : ac("move_base", true)
    {
        ac.waitForServer();
        pub = nh.advertise<assignment_2_2024::RobotState>("robot_state", 10);
        odom_sub = nh.subscribe("/odom", 10, &RobotActionClient::odomCallback, this);
    }

    void sendGoal(double x, double y)
    {
        move_base_msgs::MoveBaseGoal goal;
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
    MoveBaseClient ac;
    ros::Publisher pub;
    ros::Subscriber odom_sub;
    assignment_2_2024::RobotState robot_state;

    void doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
    {
        ROS_INFO("Goal finished with state: %s", state.toString().c_str());
    }

    void activeCb()
    {
        ROS_INFO("Goal just went active");
    }

    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
    {
        ROS_INFO("Got Feedback");
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

    // Example: send a goal (1.0, 1.0)
    client.sendGoal(1.0, 1.0);

    ros::spin();
    return 0;
}