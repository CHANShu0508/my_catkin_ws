/**
 * @file    chassis_control.cpp
 * @author  Chen Shu
 * @brief   The control of the forklift chassis
 * @version 0.1
 * @date    2022-04-11
 * 
 * @ref     https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
 */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include "control_msgs/JointControllerState.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

using namespace message_filters;
typedef geometry_msgs::TwistStamped ctrl_msgs;
typedef control_msgs::JointControllerState encoder_msgs;

class Chassis {
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    message_filters::Subscriber<ctrl_msgs> ctrl_sub_;
    message_filters::Subscriber<encoder_msgs> fl_sub_;
    message_filters::Subscriber<encoder_msgs> fr_sub_;
    message_filters::Subscriber<encoder_msgs> bl_sub_;
    message_filters::Subscriber<encoder_msgs> br_sub_;
    typedef sync_policies::ApproximateTime<ctrl_msgs, encoder_msgs, encoder_msgs, encoder_msgs, encoder_msgs> my_sync_policy_;
    typedef Synchronizer<my_sync_policy_> sync_nizer_;
    boost::shared_ptr<sync_nizer_> sync_;

public:
    void CtrlCallBack(const ctrl_msgs::ConstPtr& _ctrl_msg, 
                      const encoder_msgs::ConstPtr fl_msg,
                      const encoder_msgs::ConstPtr fr_msg,
                      const encoder_msgs::ConstPtr bl_msg,
                      const encoder_msgs::ConstPtr br_msg);

    Chassis() {
        ctrl_sub_.subscribe(nh_, "forklift/cmd_vel", 10);
        fl_sub_.subscribe(nh_, "forklift_controllers/front_left_whole_controller/state", 10);
        fr_sub_.subscribe(nh_, "forklift_controllers/front_right_whole_controller/state", 10);
        bl_sub_.subscribe(nh_, "forklift_controllers/back_left_whole_controller/state", 10);
        br_sub_.subscribe(nh_, "forklift_controllers/back_right_whole_controller/state", 10);
        sync_.reset(new sync_nizer_(my_sync_policy_(10), ctrl_sub_, fl_sub_, fr_sub_, bl_sub_, br_sub_));
        sync_->registerCallback(boost::bind(&Chassis::CtrlCallBack, this, _1, _2, _3, _4, _5));
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "forklift_chassis_control");
    Chassis forklift_chassis;

    ros::spin();
    return 0;
}

void Chassis::CtrlCallBack(const ctrl_msgs::ConstPtr& _ctrl_msg, 
                           const encoder_msgs::ConstPtr fl_msg,
                           const encoder_msgs::ConstPtr fr_msg,
                           const encoder_msgs::ConstPtr bl_msg,
                           const encoder_msgs::ConstPtr br_msg)
{
    ROS_INFO("%f\n", fl_msg->error);
}
