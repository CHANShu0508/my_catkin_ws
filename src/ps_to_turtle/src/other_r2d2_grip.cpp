#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "ds4_driver/Status.h"

class Controller2Grip {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

public:
    void CtrlStatusCallback(const ds4_driver::Status::ConstPtr& msg);

    Controller2Grip() {
        pub_ = nh_.advertise<std_msgs::Float64MultiArray>("r2d2_gripper_controller/command", 500);
        sub_ = nh_.subscribe("status", 500, &Controller2Grip::CtrlStatusCallback, this);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "other_use_ctrl_grip");
    Controller2Grip joy_stick;

    ros::spin();

    return 0;
}

void Controller2Grip::CtrlStatusCallback(const ds4_driver::Status::ConstPtr& msg)
{
    std_msgs::Float64MultiArray send_msg;
    if (msg->button_l1 == 1 && msg->button_r1 == 1) {
        send_msg.data.push_back(0.0);
    } else { send_msg.data.push_back(-0.15); }

    send_msg.data.push_back(msg->axis_l2 * 0.5);
    send_msg.data.push_back(msg->axis_r2 * 0.5);

    pub_.publish(send_msg);
    ros::spinOnce();
}
