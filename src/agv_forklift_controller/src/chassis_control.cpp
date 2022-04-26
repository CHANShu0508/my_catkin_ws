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

static bool FetchPidParam(ros::NodeHandle& _nh, Matrix<double, 4, 4>& _front, Matrix<double, 4, 4>& _back);

// --------------------------------------------- MAIN FUNCTON ---------------------------------------------
int main(int argc, char* argv[])
{
    std::vector<double> angle2velo_pid (4), velo2effort_pid (4);
    Matrix<double, 4, 4> front, back; //Temp storing
    ros::init(argc, argv, "forklift_chassis_control");

    // Get PID parameters
    ros::NodeHandle param_nh;
    bool is_fetch_succeed = FetchPidParam(param_nh, front, back);
    if (!is_fetch_succeed) {
        ROS_ERROR("No param fetch!");
        return 1;
    }

    Chassis forklift_chassis(atof(argv[1]), atof(argv[2]), front, back);
    
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

Chassis::Chassis(double _max_linear_spd, double _max_angle_spd,
                 Matrix<double, 4, 4>& _front, Matrix<double, 4, 4>& _back)
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

    front_pid_ = _front;
    back_pid_ = _back;

    front_a2v_pid_ptr_ = new Chassis_PID::PID(front_pid_(0, 0),
                                              front_pid_(0, 1),
                                              front_pid_(0, 2),
                                              front_pid_(0, 3));
    back_a2v_pid_ptr_ = new Chassis_PID::PID(back_pid_(0, 0),
                                             back_pid_(0, 1),
                                             back_pid_(0, 2),
                                             back_pid_(0, 3));
    front_v2e_pid_ptr_ = new Chassis_PID::PID_2(front_pid_(1, 0),
                                                front_pid_(1, 1),
                                                front_pid_(1, 2),
                                                front_pid_(1, 3));
    back_v2e_pid_ptr_ = new Chassis_PID::PID_2(back_pid_(1, 0),
                                               back_pid_(1, 1),
                                               back_pid_(1, 2),
                                               back_pid_(1, 3));

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
    front_a2v_pid_ptr_->impl->UpdatePID(_pid_msg->data[0], _pid_msg->data[1], _pid_msg->data[2], _pid_msg->data[3]);
    front_v2e_pid_ptr_->impl->UpdatePID(_pid_msg->data[4], _pid_msg->data[5], _pid_msg->data[6], _pid_msg->data[7]);

    back_a2v_pid_ptr_->impl->UpdatePID(_pid_msg->data[8], _pid_msg->data[9], _pid_msg->data[10], _pid_msg->data[11]);
    back_v2e_pid_ptr_->impl->UpdatePID(_pid_msg->data[12], _pid_msg->data[13], _pid_msg->data[14], _pid_msg->data[15]);

    ROS_INFO("PID Updated!");
}

void Chassis::PublishCmd()
{
    static int debug_counter;
    RowVector2d x_unit(1.0, 0.0);
    Vector4d tar_ang_mat = Vector4d::Constant(0); // First column is target angle
    Matrix<double, 4, 2> velo_norm = Matrix<double, 4, 2>::Constant(0); // Storing the sum velocity's norm. Second column is for add the result for two wheel in one unit
    std_msgs::Float64MultiArray send_msg;
    Vector4d error_vec = Vector4d::Constant(0); // Total vector for angle error
    Vector2d error_front, error_back; // Separated vector for angle error
    Matrix<double, 4, 2> a2v_result;
    Vector4d velo_error_front, velo_error_back; // Velocity error
    Matrix<double, 4, 4> is_reverse_spd_mat = Matrix<double, 4, 4>::Identity(); // By steering algorithm, does steer need reverse speed

    // Calculate the angle between the target speed direction
    if (sum_mat_.norm() < 0.057) {  // If the cmd value is too small, block it to avoid NaN
        tar_ang_mat = Vector4d::Constant(0);
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
        if (error_vec(i) > 2 * M_PI)
            error_vec(i) = 2 * M_PI - 0.01;
        else if (error_vec(i) < -2 * M_PI)
            error_vec(i) = -2 * M_PI + 0.01;// Clean the gap

        if (error_vec(i) > M_PI / 2 && error_vec(i) < M_PI * 3 / 2) {// 90~180
            virtual_tar_steer_mat_(i, 0) = tar_steer_mat_(i, 0) - M_PI;
            is_reverse_spd_mat(i, i) = -1;
        } else if (error_vec(i) > M_PI * 3 / 2) {
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
//    debug_counter++;
//    if (debug_counter >= 100) {
//        ROS_INFO("%.2f  %.2f  %.2f  %.2f", error_vec(0)/M_PI*180, error_vec(1)/M_PI*180, error_vec(2)/M_PI*180, error_vec(3)/M_PI*180);
//        debug_counter = 0;
//    }

    error_front = error_vec.head(2);
    error_back = error_vec.tail(2);

    front_a2v_pid_ptr_->Calculate(error_front);
    back_a2v_pid_ptr_->Calculate(error_back);
    a2v_result.block<2, 2>(0, 0) = front_a2v_pid_ptr_->impl->result_mat;
    a2v_result.block<2, 2>(2, 0) = back_a2v_pid_ptr_->impl->result_mat;
    a2v_result = a2v_result + velo_norm;
    a2v_result = is_reverse_spd_mat * a2v_result;
    a2v_result = a2v_result / 0.05; // turn into angle speed
    // Until now, we get the speed of every wheel we want

    a2v_result -= wheel_spd_mat_;
    velo_error_front.head(2) = a2v_result.row(0).transpose();
    velo_error_front.tail(2) = a2v_result.row(1).transpose();
    velo_error_back.head(2) = a2v_result.row(2).transpose();
    velo_error_back.tail(2) = a2v_result.row(3).transpose();
    front_v2e_pid_ptr_->Calculate(velo_error_front);
    back_v2e_pid_ptr_->Calculate(velo_error_back);
//    debug_counter++;
//    if (debug_counter >= 100) {
//        ROS_INFO("%.2f %.2f", velo2effort_pid_mat_ptr_->impl->result_mat(0, 0), velo2effort_pid_mat_ptr_->impl->result_mat(1, 0));
//        debug_counter = 0;
//    }
    send_msg.data.push_back(front_v2e_pid_ptr_->impl->result_mat(0));
    send_msg.data.push_back(front_v2e_pid_ptr_->impl->result_mat(1));
    send_msg.data.push_back(front_v2e_pid_ptr_->impl->result_mat(2));
    send_msg.data.push_back(front_v2e_pid_ptr_->impl->result_mat(3));
    send_msg.data.push_back(back_v2e_pid_ptr_->impl->result_mat(0));
    send_msg.data.push_back(back_v2e_pid_ptr_->impl->result_mat(1));
    send_msg.data.push_back(back_v2e_pid_ptr_->impl->result_mat(2));
    send_msg.data.push_back(back_v2e_pid_ptr_->impl->result_mat(3));
    wheels_pub_.publish(send_msg);
//    ROS_INFO("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f", send_msg.data[0], send_msg.data[1], send_msg.data[2], send_msg.data[3], send_msg.data[4], send_msg.data[5], send_msg.data[6], send_msg.data[7]);
}

Chassis::~Chassis()
{
    delete front_v2e_pid_ptr_;
    delete front_a2v_pid_ptr_;
    delete back_v2e_pid_ptr_;
    delete back_a2v_pid_ptr_;
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

static bool FetchPidParam(ros::NodeHandle& _nh, Matrix<double, 4, 4>& _front, Matrix<double, 4, 4>& _back)
{
    bool is_succeed1 = _nh.getParam("chassis_control/before/front/Angle2Velo/p", _front(0, 0));
    _nh.getParam("chassis_control/before/front/Angle2Velo/i", _front(0, 1));
    _nh.getParam("chassis_control/before/front/Angle2Velo/d", _front(0, 2));
    _nh.getParam("chassis_control/before/front/Angle2Velo/max", _front(0, 3));
    _nh.getParam("chassis_control/before/front/Velo2Effort/p", _front(1, 0));
    _nh.getParam("chassis_control/before/front/Velo2Effort/i", _front(1, 1));
    _nh.getParam("chassis_control/before/front/Velo2Effort/d", _front(1, 2));
    _nh.getParam("chassis_control/before/front/Velo2Effort/max", _front(1, 3));
    _nh.getParam("chassis_control/after/front/Angle2Velo/p", _front(2, 0));
    _nh.getParam("chassis_control/after/front/Angle2Velo/i", _front(2, 1));
    _nh.getParam("chassis_control/after/front/Angle2Velo/d", _front(2, 2));
    _nh.getParam("chassis_control/after/front/Angle2Velo/max", _front(2, 3));
    _nh.getParam("chassis_control/after/front/Velo2Effort/p", _front(3, 0));
    _nh.getParam("chassis_control/after/front/Velo2Effort/i", _front(3, 1));
    _nh.getParam("chassis_control/after/front/Velo2Effort/d", _front(3, 2));
    _nh.getParam("chassis_control/after/front/Velo2Effort/max", _front(3, 3));

    _nh.getParam("chassis_control/before/back/Angle2Velo/p", _back(0, 0));
    _nh.getParam("chassis_control/before/back/Angle2Velo/i", _back(0, 1));
    _nh.getParam("chassis_control/before/back/Angle2Velo/d", _back(0, 2));
    _nh.getParam("chassis_control/before/back/Angle2Velo/max", _back(0, 3));
    _nh.getParam("chassis_control/before/back/Velo2Effort/p", _back(1, 0));
    _nh.getParam("chassis_control/before/back/Velo2Effort/i", _back(1, 1));
    _nh.getParam("chassis_control/before/back/Velo2Effort/d", _back(1, 2));
    _nh.getParam("chassis_control/before/back/Velo2Effort/max", _back(1, 3));
    _nh.getParam("chassis_control/after/back/Angle2Velo/p", _back(2, 0));
    _nh.getParam("chassis_control/after/back/Angle2Velo/i", _back(2, 1));
    _nh.getParam("chassis_control/after/back/Angle2Velo/d", _back(2, 2));
    _nh.getParam("chassis_control/after/back/Angle2Velo/max", _back(2, 3));
    _nh.getParam("chassis_control/after/back/Velo2Effort/p", _back(3, 0));
    _nh.getParam("chassis_control/after/back/Velo2Effort/i", _back(3, 1));
    _nh.getParam("chassis_control/after/back/Velo2Effort/d", _back(3, 2));
    bool is_succeed2 = _nh.getParam("chassis_control/after/back/Velo2Effort/max", _back(3, 3));

    return (is_succeed1 && is_succeed2);
}
