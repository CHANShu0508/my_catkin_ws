#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "param_test");

    // Declare our test node handles
    ros::NodeHandle glo_nh;
    ros::NodeHandle bare_name_nh("bare_nh");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle pri_bare_nh("~p_b_nh");

    int param_1, param_2, param_3, param_4;
    bool ifget_param1 = glo_nh.getParam("the_param", param_1);
    bool ifget_param2 = bare_name_nh.getParam("the_param", param_2);
    bool ifget_param3 = private_nh.getParam("the_param", param_3);
    bool ifget_param4 = pri_bare_nh.getParam("the_param", param_4);

    if (ifget_param1) {
        ROS_INFO("global node handle fetch: %d", param_1);
    }
    if (ifget_param2) {
        ROS_INFO("bare name node handle fetch: %d", param_2);
    }
    if (ifget_param3) {
        ROS_INFO("private node handle fetch: %d", param_3);
    }
    if (ifget_param4) {
        ROS_INFO("~ + name node handle fetch: %d", param_4);
    }

    ros::spin();

    return 0;
}
