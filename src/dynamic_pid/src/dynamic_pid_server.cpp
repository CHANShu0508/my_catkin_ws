#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64MultiArray.h>
#include "dynamic_pid/Dynamic_pidConfig.h"

class DynamicPID {
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_ = nh_.advertise<std_msgs::Float64MultiArray>("test", 10);
    dynamic_reconfigure::Server<dynamic_pid::Dynamic_pidConfig> server_;
    dynamic_reconfigure::Server<dynamic_pid::Dynamic_pidConfig>::CallbackType function_;

public:
    DynamicPID();
    void CallBack(dynamic_pid::Dynamic_pidConfig &config, uint32_t level);
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dynamic_pid_server");
    DynamicPID dynamicPid;

    ros::spin();
    return 0;
}

DynamicPID::DynamicPID()
{
    function_ = boost::bind(&DynamicPID::CallBack, this, _1, _2);
    server_.setCallback(function_);

    pub_ = nh_.advertise<std_msgs::Float64MultiArray>("pid_update", 10);
}

void DynamicPID::CallBack(dynamic_pid::Dynamic_pidConfig &config, uint32_t level)
{
    std_msgs::Float64MultiArray send_msg;
    send_msg.data.push_back(config.f_angle2velo_p);
    send_msg.data.push_back(config.f_angle2velo_i);
    send_msg.data.push_back(config.f_angle2velo_d);
    send_msg.data.push_back(config.f_angle2velo_max);
    send_msg.data.push_back(config.f_velo2effort_p);
    send_msg.data.push_back(config.f_velo2effort_i);
    send_msg.data.push_back(config.f_velo2effort_d);
    send_msg.data.push_back(config.f_velo2effort_max);

    send_msg.data.push_back(config.b_angle2velo_p);
    send_msg.data.push_back(config.b_angle2velo_i);
    send_msg.data.push_back(config.b_angle2velo_d);
    send_msg.data.push_back(config.b_angle2velo_max);
    send_msg.data.push_back(config.b_velo2effort_p);
    send_msg.data.push_back(config.b_velo2effort_i);
    send_msg.data.push_back(config.b_velo2effort_d);
    send_msg.data.push_back(config.b_velo2effort_max);

    pub_.publish(send_msg);
}
