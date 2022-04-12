#include "chas_pid.h"

Chassis_PID::PIDImpl::PIDImpl(double kp, double ki, double kd, double _max_out)
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

void Chassis_PID::PIDImpl::Calculate(Vector4d& error)
{
    param_mat_.col(1) = param_mat_.col(1) + error;
    param_mat_.col(2) = error - param_mat_.col(0);
    param_mat_.col(0) = error;

    result_mat.col(0) = param_mat_ * factor_vec_;
    if (result_mat.col(0).lpNorm<1>() > max_out * 4) {
        result_mat.col(0) = result_mat.col(0) * (max_out / result_mat.col(0).cwiseAbs().maxCoeff());
    }

    result_mat.col(1) = -1 * result_mat.col(0);
}

Chassis_PID::PID::PID(double kp, double ki, double kd, double max_out)
{
    impl = new PIDImpl(kp, ki, kd, max_out);
}

void Chassis_PID::PID::Calculate(Vector4d& error)
{
    impl->Calculate(error);
}

Chassis_PID::PID::~PID()
{
    delete impl;
}
