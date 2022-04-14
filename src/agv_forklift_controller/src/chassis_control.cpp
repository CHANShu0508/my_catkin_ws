/**
 * @file    chassis_control.cpp
 * @author  Chen Shu
 * @brief   The control of the forklift chassis
 * @version 0.1
 * @date    2022-04-11
 * 
 * @ref     https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
 */

#include "chassis_control.h"
#include <cmath>

Matrix<double, 4, 2> ones_mat = MatrixXd::Constant(4, 2, 1.0);
Matrix<double, 4, 2> rot_ori_mat; // Origin matrice for the rotate matrice


// ---------------- MAIN FUNCTON ---------------
int main(int argc, char* argv[])
{
    std::vector<double> angle2velo_pid (4), velo2effort_pid (4);
    ros::init(argc, argv, "forklift_chassis_control");

    // Get PID parameters
    ros::NodeHandle param_nh;
    bool is_fetch_pid_1 = param_nh.getParam("pid_param/Velo2Effort/p", velo2effort_pid[0]);
    param_nh.getParam("pid_param/Velo2Effort/i", velo2effort_pid[1]);
    param_nh.getParam("pid_param/Velo2Effort/d", velo2effort_pid[2]);
    param_nh.getParam("pid_param/Velo2Effort/max", velo2effort_pid[3]);

    param_nh.getParam("pid_param/Angle2Velo/p", angle2velo_pid[0]);
    param_nh.getParam("pid_param/Angle2Velo/i", angle2velo_pid[1]);
    param_nh.getParam("pid_param/Angle2Velo/d", angle2velo_pid[2]);
    bool is_fetch_pid_2 = param_nh.getParam("pid_param/Angle2Velo/max", angle2velo_pid[3]);

    if (!(is_fetch_pid_1 && is_fetch_pid_2)) {
        ROS_ERROR("No param fetch!");
        return 1;
    }

    Chassis forklift_chassis(atof(argv[1]));
    forklift_chassis.angle2velo_pid_mat_ptr_ = new Chassis_PID::PID(angle2velo_pid[0], 
                                                                    angle2velo_pid[1], 
                                                                    angle2velo_pid[2], 
                                                                    angle2velo_pid[3]);
    forklift_chassis.velo2effort_pid_mat_ptr_ = new Chassis_PID::PID_2(velo2effort_pid[0], 
                                                                       velo2effort_pid[1], 
                                                                       velo2effort_pid[2], 
                                                                       velo2effort_pid[3]);                                                                    
    
    ros::spin();
    return 0;
}
// ---------------------------------------------

Chassis::Chassis(double _max_linear_spd) 
                :max_linear_spd_(_max_linear_spd)
{
    wheel2center = 0.9243616416;

    xy_vec_ << 0.0, 0.0;
    sum_mat_ = ones_mat;
    rot_ori_mat << 0.4327310676, -0.9015230575,
                   -0.4327310676, -0.9015230575,
                   0.4327310676, 0.9015230575,
                   -0.4327310676, 0.9015230575;
    rot_mat_ = rot_ori_mat;

    ctrl_sub_.subscribe(nh_, "forklift/cmd_vel", 10);
    joint_state_sub_.subscribe(nh_, "forklift_controllers/joint_states", 10);
    sync_.reset(new sync_nizer_(my_sync_policy_(10), ctrl_sub_, joint_state_sub_));
    sync_->registerCallback(boost::bind(&Chassis::CtrlCallBack, this, _1, _2));

    ros::Duration(0, 10000).sleep();
    wheels_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("forklift_controllers/chassis_controller/command", 10);
}

void Chassis::CtrlCallBack(const ctrl_msgs::ConstPtr& _ctrl_msg, 
                           const joint_msgs::ConstPtr& _joint_msgs)
{
    std_msgs::Float64MultiArray send_msg;
    Vector4d tar_ang_mat; // First column is target angle
    RowVector2d x_unit(1.0, 0.0);
    Matrix<double, 4, 2> velo_norm; // Storing the sum velocity's norm. Second column is for add the result for two wheel in one unit
    Vector4d error_vec;
    std::vector<bool> is_reverse_spd (4, false); // By steering algorithm, does steer need reverse speed
    bool is_shield_cmd = false;

    // Process the encoder of steering
    steer_mat_(0) = fmod(_joint_msgs->position[9], M_PI);   // Front Left
    steer_mat_(1) = fmod(_joint_msgs->position[12], M_PI);  // Front Right
    steer_mat_(2) = fmod(_joint_msgs->position[2], M_PI);   // Back Left
    steer_mat_(3) = fmod(_joint_msgs->position[5], M_PI);   // Back Right

    // First add the pure linear speed to sum
    xy_vec_(0) = _ctrl_msg->twist.linear.x;
    xy_vec_(1) = _ctrl_msg->twist.linear.y;
    if (xy_vec_.norm() > max_linear_spd_) { xy_vec_.normalize(); }
    sum_mat_.col(0) = sum_mat_.col(0) * xy_vec_(0);
    sum_mat_.col(1) = sum_mat_.col(1) * xy_vec_(1);

    // Then add the rotate speed to the sum
    rot_mat_ = rot_mat_ * (_ctrl_msg->twist.angular.z * wheel2center);
    sum_mat_ = sum_mat_ + rot_mat_;
    
    // Calculate the angle between the target speed direction
    if (sum_mat_.norm() < 0.057) {  // If the cmd value is too small, block it to avoid NaN
        tar_ang_mat = Vector4d::Constant(0);
        is_shield_cmd = true;
    } else {
        for (int i = 0; i < 4; i++) { 
            velo_norm(i, 0) = sum_mat_.row(i).norm();
            velo_norm(i, 1) = velo_norm(i, 0);
            tar_ang_mat(i) = sum_mat_.row(i).dot(x_unit) / velo_norm(i, 0);
            tar_ang_mat(i) = acosf64(tar_ang_mat(i));

            // If toward the counter clockwise, made it negative <important>
            if (sum_mat_(i, 1) > 0) { tar_ang_mat(i) = -tar_ang_mat(i); }
        }
    }

    /* For the steering:
     *   - Target angle is in the `target_ang_mat` matrix
     *   - Now the angle is from `steer_mat_`
     *   - They are both 4 dimensional column vector
     * 
     * And my method to steering control is dividing into four areas
     *   - Two areas we will rotate a small angle, and reverse the speed
     *   - Detailed in the paper
    */
    error_vec = tar_ang_mat - steer_mat_;
    for (int i = 0; i < 4; i++) {
        if (error_vec(i) > M_PI/2 && error_vec(i) < M_PI) {            // 90~180
            error_vec(i) = error_vec(i) - M_PI;
            is_reverse_spd[i] = true;
        } else if (error_vec(i) < -(M_PI/2) && error_vec(i) > -M_PI) { // -90~-180
            error_vec(i) = error_vec(i) + M_PI;
            is_reverse_spd[i] = true;
        }
    }

    angle2velo_pid_mat_ptr_->Calculate(error_vec);
    angle2velo_pid_mat_ptr_->impl->result_mat = angle2velo_pid_mat_ptr_->impl->result_mat + velo_norm;
    if (is_shield_cmd) { angle2velo_pid_mat_ptr_->impl->result_mat = Matrix<double, 4, 2>::Constant(0); } // Cmd value is too small, avoid NaN
    // Until now, we get the speed of every wheel we want

    ROS_INFO("%.2f %.2f", angle2velo_pid_mat_ptr_->impl->result_mat(0, 0), angle2velo_pid_mat_ptr_->impl->result_mat(1, 0));

    sum_mat_ = ones_mat;
    rot_mat_ = rot_ori_mat;
}
