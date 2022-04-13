#ifndef __CHAS_PID_H__
#define __CHAS_PID_H__
#include <eigen3/Eigen/Dense>
using namespace Eigen;

namespace Chassis_PID {
    class PIDImpl { // Real implement of PID class
    private:
        Vector3d factor_vec_;
        Matrix<double, 4, 3> param_mat_;
        double max_out;

    public:
        Matrix<double, 4, 2> result_mat;

        PIDImpl(double kp, double ki, double kd, double _max_out);
        void Calculate(Vector4d& error);
    };

    class PID {
    public:
        PIDImpl *impl;
        PID(double kp, double ki, double kd, double max_out);
        ~PID();
        void Calculate(Vector4d& error);
    };
}
#endif /* __CHAS_PID_H__ */
