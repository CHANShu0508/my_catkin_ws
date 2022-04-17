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

Matrix<double, 4, 2> ones_mat = Matrix<double, 4, 2>::Constant(1.0);
Matrix<double, 4, 2> rot_ori_mat; // Origin matrices for the rotates matrice


// --------------------------------------------- MAIN FUNCTON ---------------------------------------------
int main(int argc, char* argv[])
{
    std::vector<double> angle2velo_pid (4), velo2effort_pid (4);
    ros::init(argc, argv, "forklift_chassis_control");

    // Get PID parameters
    ros::NodeHandle param_nh;
    bool is_fetch_pid_1 = param_nh.getParam("chassis_control/Velo2Effort/p", velo2effort_pid[0]);
    param_nh.getParam("chassis_control/Velo2Effort/i", velo2effort_pid[1]);
    param_nh.getParam("chassis_control/Velo2Effort/d", velo2effort_pid[2]);
    param_nh.getParam("chassis_control/Velo2Effort/max", velo2effort_pid[3]);

    param_nh.getParam("chassis_control/Angle2Velo/p", angle2velo_pid[0]);
    param_nh.getParam("chassis_control/Angle2Velo/i", angle2velo_pid[1]);
    param_nh.getParam("chassis_control/Angle2Velo/d", angle2velo_pid[2]);
    bool is_fetch_pid_2 = param_nh.getParam("chassis_control/Angle2Velo/max", angle2velo_pid[3]);

    if (!(is_fetch_pid_1 && is_fetch_pid_2)) {
        ROS_ERROR("No param fetch!");
        return 1;
    }

    Chassis forklift_chassis(atof(argv[1]), atof(argv[2]));
    forklift_chassis.angle2velo_pid_mat_ptr_ = new Chassis_PID::PID(angle2velo_pid[0], 
                                                                    angle2velo_pid[1], 
                                                                    angle2velo_pid[2], 
                                                                    angle2velo_pid[3]);
    forklift_chassis.velo2effort_pid_mat_ptr_ = new Chassis_PID::PID_2(velo2effort_pid[0], 
                                                                       velo2effort_pid[1], 
                                                                       velo2effort_pid[2], 
                                                                       velo2effort_pid[3]);
    
    ros::Duration(0, 10000).sleep();
    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        forklift_chassis.PublishCmd();

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
// --------------------------------------------------------------------------------------------------------

Chassis::Chassis(double _max_linear_spd, double _max_angle_spd)
                :max_linear_spd_(_max_linear_spd), max_angle_spd_(_max_angle_spd)
{
    wheel2center = 0.9243616416;
    xy_vec_ << 0.0, 0.0;
    sum_mat_ = ones_mat;
    rot_ori_mat << 0.4327310676, -0.9015230575,
                   -0.4327310676, -0.9015230575,
                   0.4327310676, 0.9015230575,
                   -0.4327310676, 0.9015230575;
    rot_mat_ = rot_ori_mat;

    pid_update_sub_ = nh_.subscribe("pid_update", 10, &Chassis::PidUpdateCallback, this);
    ctrl_sub_.subscribe(nh_, "forklift/cmd_vel", 10);
    joint_state_sub_.subscribe(nh_, "forklift_controllers/joint_states", 10);
    sync_.reset(new sync_nizer_(my_sync_policy_(10), ctrl_sub_, joint_state_sub_));
    sync_->registerCallback(boost::bind(&Chassis::CtrlCallBack, this, _1, _2));

    wheels_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("forklift_controllers/chassis_controller/command", 1);
}

void Chassis::CtrlCallBack(const ctrl_msgs::ConstPtr& _ctrl_msg, 
                           const joint_msgs::ConstPtr& _joint_msgs)
{
    // Record wheel speeds (angle speed)
    wheel_spd_mat_(0, 0) = _joint_msgs->velocity[7];
    wheel_spd_mat_(0, 1) = _joint_msgs->velocity[8];
    wheel_spd_mat_(1, 0) = _joint_msgs->velocity[10];
    wheel_spd_mat_(1, 1) = _joint_msgs->velocity[11];
    wheel_spd_mat_(2, 0) = _joint_msgs->velocity[0];
    wheel_spd_mat_(2, 1) = _joint_msgs->velocity[1];
    wheel_spd_mat_(3, 0) = _joint_msgs->velocity[3];
    wheel_spd_mat_(3, 1) = _joint_msgs->velocity[4];

    // Process the encoder of steering
    ContinueProcessSteer(_joint_msgs->position[9], steer_mat_, 0);   // Front Left
    ContinueProcessSteer(_joint_msgs->position[12], steer_mat_, 1);  // Front Right
    ContinueProcessSteer(_joint_msgs->position[2], steer_mat_, 2);   // Back Left
    ContinueProcessSteer(_joint_msgs->position[5], steer_mat_, 3);   // Back Right

    // First add the pure linear speed to sum
    xy_vec_(0) = _ctrl_msg->twist.linear.x;
    xy_vec_(1) = _ctrl_msg->twist.linear.y;
    if (xy_vec_.norm() > max_linear_spd_) { xy_vec_.normalize(); }
    sum_mat_.col(0) = ones_mat.col(0) * xy_vec_(0);
    sum_mat_.col(1) = ones_mat.col(1) * xy_vec_(1);

    // Then add the rotates speed to the sum
    rot_mat_ = rot_ori_mat * (_ctrl_msg->twist.angular.z * wheel2center);
    sum_mat_ = sum_mat_ + rot_mat_;
    // ROS_INFO("(%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f)", sum_mat_(0, 0), sum_mat_(0, 1), sum_mat_(1, 0), sum_mat_(1, 1), sum_mat_(2, 0), sum_mat_(2, 1), sum_mat_(3, 0), sum_mat_(3, 1));
}

void Chassis::PidUpdateCallback(const std_msgs::Float64MultiArray::ConstPtr &_pid_msg)
{
    angle2velo_pid_mat_ptr_->impl->UpdatePID(_pid_msg->data[0], _pid_msg->data[1], _pid_msg->data[2], _pid_msg->data[3]);
    velo2effort_pid_mat_ptr_->impl->UpdatePID(_pid_msg->data[4], _pid_msg->data[5], _pid_msg->data[6], _pid_msg->data[7]);

    ROS_INFO("PID Updated!");
}

void Chassis::PublishCmd()
{
    static int debug_counter;
    RowVector2d x_unit(1.0, 0.0);
    Vector4d tar_ang_mat = Vector4d::Constant(0); // First column is target angle
    bool is_shield_cmd = false;
    Matrix<double, 4, 2> velo_norm = Matrix<double, 4, 2>::Constant(0); // Storing the sum velocity's norm. Second column is for add the result for two wheel in one unit
    std_msgs::Float64MultiArray send_msg;
    Vector4d error_vec = Vector4d::Constant(0); 
    Matrix<double, 8, 1> velo_error_vec = Matrix<double, 8, 1>::Constant(0); // 0~3 is all left side of each unit; 4~8 is all right side of each unit
    Matrix<double, 4, 4> is_reverse_spd_mat = Matrix<double, 4, 4>::Identity(); // By steering algorithm, does steer need reverse speed

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

            // If toward the counterclockwise, made it continous <important>
            if (sum_mat_(i, 1) > 0) { tar_ang_mat(i) = 2*M_PI - tar_ang_mat(i); }
        }
    }
    for (int i = 0; i < 4; i++)  { GetRealTarget(tar_steer_mat_, tar_ang_mat(i), int(steer_mat_(i, 2)), i); }
//    debug_counter++;
//    if (debug_counter >= 100) {
//        ROS_INFO("%.2f  %.2f  %.2f  %.2f", tar_ang_mat(0)/M_PI*180, tar_ang_mat(1)/M_PI*180, tar_ang_mat(2)/M_PI*180, tar_ang_mat(3)/M_PI*180);
//        debug_counter = 0;
//    }
//    debug_counter++;
//    if (debug_counter >= 100) {
//        ROS_INFO("%.2f  %.2f  %.2f  %.2f", steer_mat_(0, 1)/M_PI*180, steer_mat_(1, 1)/M_PI*180, steer_mat_(2, 1)/M_PI*180, steer_mat_(3, 1)/M_PI*180);
//        debug_counter = 0;
//    }

    /* For the steering:
     *   - Target angle is in the `target_ang_mat` matrix
     *   - Now the angle is from `steer_mat_`
     *   - They are both 4 dimensional column vector
     * 
     * And my method to steering control is dividing into four areas
     *   - Two areas we will rotate a small angle, and reverse the speed
     *   - Detailed in the paper
    */
    error_vec = tar_steer_mat_.col(1) - steer_mat_.col(1);
    for (int i = 0; i < 4; i++) {
        if (error_vec(i) > M_PI/2 && error_vec(i) < M_PI*3/2) {// 90~180
            virtual_tar_steer_mat_(i, 0) = tar_steer_mat_(i, 0) - M_PI;
            is_reverse_spd_mat(i, i) = -1;
        } else if (error_vec(i) > M_PI*3/2) {
            virtual_tar_steer_mat_(i, 0) = tar_steer_mat_(i, 0) - 2 * M_PI;
        } else if (error_vec(i) < -(M_PI / 2) && error_vec(i) > -(M_PI * 3 / 2)) {// -90~-180
            virtual_tar_steer_mat_(i, 0) = tar_steer_mat_(i, 0) + M_PI;
            is_reverse_spd_mat(i, i) = -1;
        } else if (error_vec(i) < -(M_PI * 3 / 2)) {
            virtual_tar_steer_mat_(i, 0) = tar_steer_mat_(i, 0) + 2 * M_PI;
        } else {
            virtual_tar_steer_mat_(i, 0) = tar_steer_mat_(i, 0);
        }
    }
    error_vec = virtual_tar_steer_mat_.col(0) - steer_mat_.col(0);
    debug_counter++;
    if (debug_counter >= 100) {
        ROS_INFO("%.2f  %.2f  %.2f  %.2f", error_vec(0)/M_PI*180, error_vec(1)/M_PI*180, error_vec(2)/M_PI*180, error_vec(3)/M_PI*180);
        debug_counter = 0;
    }

    angle2velo_pid_mat_ptr_->Calculate(error_vec);
    // angle2velo_pid_mat_ptr_->impl->result_mat = angle2velo_pid_mat_ptr_->impl->result_mat + velo_norm;
    angle2velo_pid_mat_ptr_->impl->result_mat = is_reverse_spd_mat * angle2velo_pid_mat_ptr_->impl->result_mat;
    angle2velo_pid_mat_ptr_->impl->result_mat = angle2velo_pid_mat_ptr_->impl->result_mat / 0.05; // turn into angle speed
    // Until now, we get the speed of every wheel we want

    velo_error_vec.head(4) = angle2velo_pid_mat_ptr_->impl->result_mat.col(0) - wheel_spd_mat_.col(0);
    velo_error_vec.tail(4) = angle2velo_pid_mat_ptr_->impl->result_mat.col(1) - wheel_spd_mat_.col(1);
    velo2effort_pid_mat_ptr_->Calculate(velo_error_vec);
//    debug_counter++;
//    if (debug_counter >= 100) {
//        ROS_INFO("%.2f %.2f", velo2effort_pid_mat_ptr_->impl->result_mat(0, 0), velo2effort_pid_mat_ptr_->impl->result_mat(1, 0));
//        debug_counter = 0;
//    }
    send_msg.data.push_back(velo2effort_pid_mat_ptr_->impl->result_mat(0));
    send_msg.data.push_back(velo2effort_pid_mat_ptr_->impl->result_mat(4));
    send_msg.data.push_back(velo2effort_pid_mat_ptr_->impl->result_mat(1));
    send_msg.data.push_back(velo2effort_pid_mat_ptr_->impl->result_mat(5));
    send_msg.data.push_back(velo2effort_pid_mat_ptr_->impl->result_mat(2));
    send_msg.data.push_back(velo2effort_pid_mat_ptr_->impl->result_mat(6));
    send_msg.data.push_back(velo2effort_pid_mat_ptr_->impl->result_mat(3));
    send_msg.data.push_back(velo2effort_pid_mat_ptr_->impl->result_mat(7));
    wheels_pub_.publish(send_msg);
//    ROS_INFO("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f", send_msg.data[0], send_msg.data[1], send_msg.data[2], send_msg.data[3], send_msg.data[4], send_msg.data[5], send_msg.data[6], send_msg.data[7]);
}

Chassis::~Chassis()
{
    delete angle2velo_pid_mat_ptr_;
    delete velo2effort_pid_mat_ptr_;
}

void Chassis::ContinueProcessSteer(double _angle, Matrix<double, 4, 3>& _mat, int _num)
{
    double mid_result = _angle / (2*M_PI);
    int mid_int = int(mid_result);
    double rest = _angle - 2 * M_PI * double(mid_int);

    _mat(_num, 0) = _angle;
    _mat(_num, 1)  = rest;
    _mat(_num, 2) = double(mid_int);
}

void Chassis::GetRealTarget(Matrix<double, 4, 3>& _mat, double _angle, int _circle, int _num)
{
    _mat(_num, 0) = _angle + double(_circle)*M_PI*2;
    _mat(_num, 1) = _angle;
    _mat(_num, 2) = double(_circle);
}
