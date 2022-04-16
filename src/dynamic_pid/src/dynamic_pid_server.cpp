#include <ros/ros.h>
#include "dynamic_reconfigure/server.h"
#include "dynamic_pid/Dynamic_pidConfig.h"

void CallBack(dynamic_pid::Dynamic_pidConfig &config, uint32_t level)
{
    ROS_INFO("P: %.2f  I: %.2f  D: %.2f", config.p, config.i, config.d);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dynamic_pid_server");

    dynamic_reconfigure::Server<dynamic_pid::Dynamic_pidConfig> server;
    dynamic_reconfigure::Server<dynamic_pid::Dynamic_pidConfig>::CallbackType function;

    function = boost::bind(&CallBack, _1, _2);
    server.setCallback(function);

    ros::spin();
    return 0;
}
