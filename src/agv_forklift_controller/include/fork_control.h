#ifndef __FORK_CONTROL_H__
#define __FORK_CONTROL_H__
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

class ForkCtrl {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher cmd_pub_;

    double fork_cmd_val;
    double fork_position;

public:
    ForkCtrl();
    void CmdCallback(const std_msgs::Float64MultiArrayConstPtr &_ptr);
    void PublishData();
};

#endif /* __FORK_CONTROL_H__ */
