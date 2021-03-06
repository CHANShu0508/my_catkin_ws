#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ds4_driver/Status.h"

double translation_vel_factor, rotation_vel_factor;

class Controller2Cmd {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

public:
    void CtrlStatusCallback(const ds4_driver::Status::ConstPtr& msg);

    Controller2Cmd(std::string _prefix) {
        pub_ = nh_.advertise<geometry_msgs::Twist>(_prefix + "/cmd_vel", 100);
        sub_ = nh_.subscribe("status", 200, &Controller2Cmd::CtrlStatusCallback, this);
    }
};

int main(int argc, char *argv[])
{
    std::string cmd_prefix;
    bool is_get_prefix;

    ros::init(argc, argv, "controller_to_twist");
    if (argc != 4) {
        ROS_WARN("Usage: controller_cmd <cmd_prefix> <tranlation_factor> <rotation_factor>");
    } else { 
        cmd_prefix = argv[1];
        translation_vel_factor = atof(argv[2]);
        rotation_vel_factor = atof(argv[3]);
    }

    Controller2Cmd joy_stick(cmd_prefix);
    ros::spin();

    return 0;
}

void Controller2Cmd::CtrlStatusCallback(const ds4_driver::Status::ConstPtr& msg)
{
    geometry_msgs::Twist send_msg;
    send_msg.linear.x = translation_vel_factor * msg->axis_left_y;
    send_msg.linear.y = translation_vel_factor * msg->axis_left_x;
    send_msg.linear.z = 0.0;
    send_msg.angular.x = 0.0;
    send_msg.angular.y = 0.0;
    send_msg.angular.z = rotation_vel_factor * msg->axis_right_x;

    pub_.publish(send_msg);
    ros::spinOnce();
}
