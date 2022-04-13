/**
 * @file    chassis_control.cpp
 * @author  Chen Shu
 * @brief   The control of the forklift chassis
 * @version 0.1
 * @date    2022-04-11
 * 
 * @ref     https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
 */

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/TwistStamped.h"
#include "control_msgs/JointControllerState.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "chas_pid.h"

using namespace message_filters;
using namespace Eigen;
typedef geometry_msgs::TwistStamped ctrl_msgs;
typedef control_msgs::JointControllerState encoder_msgs;

Matrix<double, 4, 2> ones_mat = MatrixXd::Constant(4, 2, 1.0);
Matrix<double, 4, 2> rot_ori_mat; // Origin matrice for the rotate matrice

class Chassis {
private:
    ros::NodeHandle nh_;
    ros::Publisher wheels_pub_;
    message_filters::Subscriber<ctrl_msgs> ctrl_sub_;
    message_filters::Subscriber<encoder_msgs> fl_sub_;
    message_filters::Subscriber<encoder_msgs> fr_sub_;
    message_filters::Subscriber<encoder_msgs> bl_sub_;
    message_filters::Subscriber<encoder_msgs> br_sub_;
    typedef sync_policies::ApproximateTime<ctrl_msgs, encoder_msgs, encoder_msgs, encoder_msgs, encoder_msgs> my_sync_policy_;
    typedef Synchronizer<my_sync_policy_> sync_nizer_;
    boost::shared_ptr<sync_nizer_> sync_;

    Vector2d xy_vec_;
    Matrix<double, 4, 2> sum_mat_; // The matrix to store the result of linear speed + rotate vector of four wheels
    Matrix<double, 4, 2> rot_mat_;
    Chassis_PID::PID *pid_mat_ptr_; // PID matrix for steer changing

    double wheel2center;

public:
    void CtrlCallBack(const ctrl_msgs::ConstPtr& _ctrl_msg, 
                      const encoder_msgs::ConstPtr fl_msg,
                      const encoder_msgs::ConstPtr fr_msg,
                      const encoder_msgs::ConstPtr bl_msg,
                      const encoder_msgs::ConstPtr br_msg);

    Chassis(double k_p, double k_i, double k_d, double max_out) {
        wheel2center = 0.9243616416;

        xy_vec_ << 0.0, 0.0;
        sum_mat_ = ones_mat;
        rot_ori_mat << -0.4327310676, 0.9015230575,
                       0.4327310676, 0.9015230575,
                       -0.4327310676, -0.9015230575,
                       0.4327310676, -0.9015230575;
        rot_mat_ = rot_ori_mat;

        pid_mat_ptr_ = new Chassis_PID::PID(k_p, k_i, k_d, max_out);

        ctrl_sub_.subscribe(nh_, "forklift/cmd_vel", 10);
        fl_sub_.subscribe(nh_, "forklift_controllers/front_left_whole_controller/state", 10);
        fr_sub_.subscribe(nh_, "forklift_controllers/front_right_whole_controller/state", 10);
        bl_sub_.subscribe(nh_, "forklift_controllers/back_left_whole_controller/state", 10);
        br_sub_.subscribe(nh_, "forklift_controllers/back_right_whole_controller/state", 10);
        sync_.reset(new sync_nizer_(my_sync_policy_(10), ctrl_sub_, fl_sub_, fr_sub_, bl_sub_, br_sub_));
        sync_->registerCallback(boost::bind(&Chassis::CtrlCallBack, this, _1, _2, _3, _4, _5));

        ros::Duration(0, 10000).sleep();
        wheels_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("forklift_controllers/chassis_controller/command", 10);
    }

    ~Chassis() {
        delete pid_mat_ptr_;
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "forklift_chassis_control");
    Chassis forklift_chassis(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]));

    ros::spin();
    return 0;
}

void Chassis::CtrlCallBack(const ctrl_msgs::ConstPtr& _ctrl_msg, 
                           const encoder_msgs::ConstPtr fl_msg,
                           const encoder_msgs::ConstPtr fr_msg,
                           const encoder_msgs::ConstPtr bl_msg,
                           const encoder_msgs::ConstPtr br_msg)
{
    std_msgs::Float64MultiArray send_msg;
    Matrix<double, 4, 3> tar_now_mat;
    RowVector2d x_uint(1.0, 0.0);
    Matrix<double, 4, 2> velo_norm;
    Vector4d error_vec;

    xy_vec_(0) = _ctrl_msg->twist.linear.x;
    xy_vec_(1) = _ctrl_msg->twist.linear.y;
    if (xy_vec_.norm() > 1.0) { xy_vec_.normalize(); }

    sum_mat_.col(0) = sum_mat_.col(0) * xy_vec_(0);
    sum_mat_.col(1) = sum_mat_.col(1) * xy_vec_(1);
    rot_mat_ = rot_mat_ * (_ctrl_msg->twist.angular.z * wheel2center);
    sum_mat_ = sum_mat_ + rot_mat_;
    
    for (int i = 0; i < 4; i++) { 
        velo_norm(0, 0) = sum_mat_.row(i).norm();
        velo_norm(0, 1) = velo_norm(0, 0);
        tar_now_mat(i, 0) = sum_mat_.row(i).dot(x_uint) / velo_norm(0, 0); 
        tar_now_mat(i, 0) = acosf64(tar_now_mat(i, 0));
    }

    tar_now_mat(0, 1) = fl_msg->error;
    tar_now_mat(1, 1) = fr_msg->error;
    tar_now_mat(2, 1) = bl_msg->error;
    tar_now_mat(3, 1) = br_msg->error;
    tar_now_mat.col(2) = tar_now_mat.col(0) - tar_now_mat.col(1);

    for (int i = 0; i < 4; i++) {
        if (tar_now_mat(i, 2) > M_PI)       tar_now_mat(i, 2) -= M_PI;
        else if (tar_now_mat(i, 2) < -M_PI) tar_now_mat(i, 2) += M_PI;
    }
    error_vec = tar_now_mat.col(2);
    pid_mat_ptr_->Calculate(error_vec);

    pid_mat_ptr_->impl->result_mat = pid_mat_ptr_->impl->result_mat + velo_norm;

    // send_msg.data.push_back(pid_mat_ptr_->impl->result_mat(0, 0));
    // send_msg.data.push_back(pid_mat_ptr_->impl->result_mat(0, 1));
    // send_msg.data.push_back(pid_mat_ptr_->impl->result_mat(1, 0));
    // send_msg.data.push_back(pid_mat_ptr_->impl->result_mat(1, 1));
    // send_msg.data.push_back(pid_mat_ptr_->impl->result_mat(2, 0));
    // send_msg.data.push_back(pid_mat_ptr_->impl->result_mat(2, 1));
    // send_msg.data.push_back(pid_mat_ptr_->impl->result_mat(3, 0));
    // send_msg.data.push_back(pid_mat_ptr_->impl->result_mat(3, 1));

    send_msg.data.push_back(20.0);
    send_msg.data.push_back(20.0);
    send_msg.data.push_back(20.0);
    send_msg.data.push_back(20.0);
    send_msg.data.push_back(20.0);
    send_msg.data.push_back(20.0);
    send_msg.data.push_back(20.0);
    send_msg.data.push_back(20.0);

    printf("%.2f %.2f\n", pid_mat_ptr_->impl->result_mat(0, 0), pid_mat_ptr_->impl->result_mat(3, 1));
    // wheels_pub_.publish(send_msg);

    sum_mat_ = ones_mat;
    rot_mat_ = rot_ori_mat;
}
