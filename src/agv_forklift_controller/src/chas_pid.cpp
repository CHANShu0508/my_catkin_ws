#include "chas_pid.h"
#include <vector>

Chassis_PID::PIDImpl::PIDImpl(double kp, double ki, double kd, double _max_out)
{
    param_mat_ << 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0;
    factor_vec_(0) = kp;
    factor_vec_(1) = ki;
    factor_vec_(2) = kd;
    max_out = _max_out;
}

void Chassis_PID::PIDImpl::Calculate(Vector2d& error)
{
    static std::vector<int> counter (2, 0);
    // Process of the parameter matrix
    param_mat_.col(1) = param_mat_.col(1) + error;
    param_mat_.col(2) = error - param_mat_.col(0);
    param_mat_.col(0) = error;
    for (int i = 0; i < 2; i++) {
        if (fabs(error(i)) > 0.06) {
            counter[i] = 0;
        } else {
            ++counter[i];
            if (counter[i] > 500)    { param_mat_(i, 1) = 0; counter[i] = 0; }
        }
    }

    /* Limit the max value:  
     *   - First calculate the sum of absolute values of this result column
     *   - If one number is greater than max value, all times smaller factor
    */
    result_mat.col(0) = param_mat_ * factor_vec_;
    if (result_mat.col(0).lpNorm<1>() > max_out * 2) {
        result_mat.col(0) = result_mat.col(0) * (max_out / result_mat.col(0).cwiseAbs().maxCoeff());
    }

    result_mat.col(1) = -1 * result_mat.col(0);
}

void Chassis_PID::PIDImpl::UpdatePID(double _p, double _i, double _d, double _max)
{
    factor_vec_ << _p, _i, _d;
    max_out = _max;
}

Chassis_PID::PID::PID(double kp, double ki, double kd, double max_out)
{
    impl = new PIDImpl(kp, ki, kd, max_out);
}

void Chassis_PID::PID::Calculate(Vector2d& error)
{
    impl->Calculate(error);
}

Chassis_PID::PID::~PID()
{
    delete impl;
}

//--------------------------------------------------------------------------
Chassis_PID::PIDImpl_2::PIDImpl_2(double kp, double ki, double kd, double _max_out)
{
    param_mat_ << 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0;
    factor_vec_(0) = kp;
    factor_vec_(1) = ki;
    factor_vec_(2) = kd;
    max_out = _max_out;
}

void Chassis_PID::PIDImpl_2::Calculate(Matrix<double, 4, 1>& error)
{
    static std::vector<int> counter (8, 0);
    // Process of the parameter matrix
    param_mat_.col(1) = param_mat_.col(1) + error;
    param_mat_.col(2) = error - param_mat_.col(0);
    param_mat_.col(0) = error;
    for (int i = 0; i < 4; i++) {
        if (fabs(error(i)) > 0.1) {
            counter[i] = 0;
        } else {
            ++counter[i];
            if (counter[i] > 500)    { param_mat_(i, 1) = 0; counter[i] = 0; }
        }
    }

    /* Limit the max value:  
     *   - First calculate the sum of absolute values of this result column
     *   - If one number is greater than max value, all times smaller factor
    */
    result_mat.col(0) = param_mat_ * factor_vec_;
    if (result_mat.col(0).lpNorm<1>() > max_out * 4) {
        result_mat.col(0) = result_mat.col(0) * (max_out / result_mat.col(0).cwiseAbs().maxCoeff());
    }
}

void Chassis_PID::PIDImpl_2::UpdatePID(double _p, double _i, double _d, double _max)
{
    factor_vec_ << _p, _i, _d;
    max_out = _max;
}

Chassis_PID::PID_2::PID_2(double kp, double ki, double kd, double max_out)
{
    impl = new PIDImpl_2(kp, ki, kd, max_out);
}

void Chassis_PID::PID_2::Calculate(Matrix<double, 4, 1>& error)
{
    impl->Calculate(error);
}

Chassis_PID::PID_2::~PID_2()
{
    delete impl;
}
