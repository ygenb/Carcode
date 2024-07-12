#include "planning/plan_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plan_test_node");
    ros::NodeHandle nh_;
    
    PlanManager plan_manager(nh_);

    ros::spin();

    return 0;
}
