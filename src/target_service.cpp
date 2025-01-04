#include <ros/ros.h>
#include <assignment_2_2024/GetTarget.h>

class TargetService
{
public:
    TargetService() : nh("~")
    {
        service = nh.advertiseService("get_target", &TargetService::handleGetTarget, this);
    }

    bool handleGetTarget(assignment_2_2024::GetTarget::Request &req, assignment_2_2024::GetTarget::Response &res)
    {
        res.x = current_goal.first;
        res.y = current_goal.second;
        return true;
    }

    void setGoal(double x, double y)
    {
        current_goal = std::make_pair(x, y);
    }

private:
    ros::NodeHandle nh;
    ros::ServiceServer service;
    std::pair<double, double> current_goal;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_service");
    TargetService service;

    // Example to set a goal (1.0, 1.0)
    service.setGoal(1.0, 1.0);

    ros::spin();
    return 0;
}
