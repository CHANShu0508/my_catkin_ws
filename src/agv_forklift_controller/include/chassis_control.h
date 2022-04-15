#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "chas_pid.h"

using namespace message_filters;
using namespace Eigen;
typedef geometry_msgs::TwistStamped ctrl_msgs;
typedef sensor_msgs::JointState joint_msgs;

class Chassis {
private:
    // ROS node settings
    ros::NodeHandle nh_;
    ros::Publisher wheels_pub_;
    message_filters::Subscriber<ctrl_msgs> ctrl_sub_;
    message_filters::Subscriber<joint_msgs> joint_state_sub_;
    typedef sync_policies::ApproximateTime<ctrl_msgs, joint_msgs> my_sync_policy_;
    typedef Synchronizer<my_sync_policy_> sync_nizer_;
    boost::shared_ptr<sync_nizer_> sync_;

    double max_linear_spd_;  // The max speed of the whole forklift
    double max_angle_spd_;
    Vector2d xy_vec_;  // The command speed direction vector
    Matrix<double, 4, 2> sum_mat_; // The matrix to store the result of linear speed + rotate vector of four wheels
    Matrix<double, 4, 2> rot_mat_;
    Matrix<double, 4, 2> wheel_spd_mat_;  // Matrix storing eight wheels' angle speed
    Vector4d steer_mat_;  // The matrix storing steering angle; First column is now, second column is last time
    double wheel2center;  // Distance between body center and wheel center

public:
    Chassis_PID::PID *angle2velo_pid_mat_ptr_;  // PID matrix for steer changing
    Chassis_PID::PID_2 *velo2effort_pid_mat_ptr_; // PID matrix for velocity to effort
    void CtrlCallBack(const ctrl_msgs::ConstPtr& _ctrl_msg, 
                      const joint_msgs::ConstPtr& _joint_msgs);
    void PublishCmd();

    Chassis(double _max_linear_spd);
    ~Chassis();
};

#endif /* __CHASSIS_CONTROL_H__ */
