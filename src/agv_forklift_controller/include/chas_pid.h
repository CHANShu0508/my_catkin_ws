#ifndef __CHAS_PID_H__
#define __CHAS_PID_H__
#include <eigen3/Eigen/Dense>
using namespace Eigen;

namespace Chassis_PID {
    class PIDImpl { // Real implement of PID class
    private:
        Vector3d factor_vec_;  // PID factor vector
        Matrix<double, 4, 3> param_mat_;
        double max_out;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
        Matrix<double, 4, 2> result_mat;

        PIDImpl(double kp, double ki, double kd, double _max_out);
        void Calculate(Vector4d& error);
        void UpdatePID(double _p, double _i, double _d, double _max);
    };

    class PID {
    public:
        PIDImpl *impl;
        PID(double kp, double ki, double kd, double max_out);
        ~PID();
        void Calculate(Vector4d& error);
    };

    class PIDImpl_2 { // Real implement of PID class
    private:
        Vector3d factor_vec_;  // PID factor vector
        Matrix<double, 8, 3> param_mat_;
        double max_out;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
        Matrix<double, 8, 1> result_mat;

        PIDImpl_2(double kp, double ki, double kd, double _max_out);
        void Calculate(Matrix<double, 8, 1>& error);
        void UpdatePID(double _p, double _i, double _d, double _max);
    };

    class PID_2 {
    public:
        PIDImpl_2 *impl;
        PID_2(double kp, double ki, double kd, double max_out);
        ~PID_2();
        void Calculate(Matrix<double, 8, 1>& error);
    };
}
#endif /* __CHAS_PID_H__ */
