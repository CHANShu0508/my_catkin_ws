#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "ds4_driver/Status.h"
#include "std_msgs/Float64MultiArray.h"

float translation_vel_factor, rotation_vel_factor;

class Controller2Cmd {
private:
    ros::NodeHandle nh_; //ROS node handler
    ros::Subscriber sub_;// Subscriber subscribing to the PS4 controller topic
    ros::Subscriber nav_sub_;
    ros::Publisher pub_;
    ros::Publisher fork_pub_;
    ds4_driver::Status status_;
    geometry_msgs::Twist nav_cmd_;
    enum CtrlMode { MANUAL = 0,
                    NAV = 1 } mode;

public:
    void CtrlStatusCallback(const ds4_driver::Status::ConstPtr &msg);
    void NavCtrlCallback(const geometry_msgs::Twist::ConstPtr &_ptr);
    void PublishData();
    void DetectModeSwitch(int32_t _button);

    Controller2Cmd(const std::string _prefix)
    {
        fork_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("forklift/fork_cmd", 10);
        pub_ = nh_.advertise<geometry_msgs::TwistStamped>(_prefix + "/cmd_vel", 100);
        sub_ = nh_.subscribe("status", 200, &Controller2Cmd::CtrlStatusCallback, this);
        nav_sub_ = nh_.subscribe("cmd_vel", 20, &Controller2Cmd::NavCtrlCallback, this);

        mode = NAV;
    }
};

int main(int argc, char *argv[])
{
    std::string cmd_prefix;

    ros::init(argc, argv, "controller_to_twist");
    if (argc != 4) {
        ROS_WARN("Usage: controller_cmd <cmd_prefix> <tranlation_factor> <rotation_factor>");
    } else {
        cmd_prefix = argv[1];
        translation_vel_factor = (float) atof(argv[2]);
        rotation_vel_factor = (float) atof(argv[3]);
    }

    Controller2Cmd joy_stick(cmd_prefix);

    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        joy_stick.PublishData();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void Controller2Cmd::CtrlStatusCallback(const ds4_driver::Status::ConstPtr &msg)
{
    DetectModeSwitch(msg->button_ps);

    status_.axis_left_y = translation_vel_factor * msg->axis_left_y;
    status_.axis_left_x = translation_vel_factor * msg->axis_left_x;
    status_.axis_right_x = translation_vel_factor * msg->axis_right_x;
    status_.axis_l2 = msg->axis_l2;
    status_.axis_r2 = msg->axis_r2;
}

void Controller2Cmd::NavCtrlCallback(const geometry_msgs::Twist::ConstPtr &_ptr)
{
    nav_cmd_.linear.x = _ptr->linear.x;
    nav_cmd_.linear.y = _ptr->linear.y;
    nav_cmd_.linear.z = _ptr->linear.z;

    nav_cmd_.angular.x = 0.0;
    nav_cmd_.angular.y = 0.0;
    nav_cmd_.angular.z = _ptr->angular.z;
}

void Controller2Cmd::PublishData()
{
    static uint32_t counter = 0;
    static uint8_t fork_cmd_counter = 0;
    geometry_msgs::TwistStamped send_msg;
    std_msgs::Float64MultiArray fork_cmd_msg;

    if (mode == MANUAL) {
        send_msg.header.seq = counter++;
        send_msg.header.stamp = ros::Time::now();
        send_msg.twist.linear.x = status_.axis_left_y;
        send_msg.twist.linear.y = status_.axis_left_x;
        send_msg.twist.linear.z = 0.0;
        send_msg.twist.angular.x = 0.0;
        send_msg.twist.angular.y = 0.0;
        send_msg.twist.angular.z = -status_.axis_right_x;
    } else {
        send_msg.header.seq = counter++;
        send_msg.header.stamp = ros::Time::now();
        send_msg.twist.linear.x = nav_cmd_.linear.x;
        send_msg.twist.linear.y = nav_cmd_.linear.y;
        send_msg.twist.linear.z = 0.0;
        send_msg.twist.angular.x = 0.0;
        send_msg.twist.angular.y = 0.0;
        send_msg.twist.angular.z = -nav_cmd_.angular.z;
    }

    pub_.publish(send_msg);

    fork_cmd_counter++;
    if (fork_cmd_counter >= 10) {
        fork_cmd_msg.data.push_back(static_cast<double>(status_.axis_l2));
        fork_cmd_msg.data.push_back(static_cast<double>(status_.axis_r2));

        fork_pub_.publish(fork_cmd_msg);
        fork_cmd_counter = 0;
    }
}

void Controller2Cmd::DetectModeSwitch(int32_t _button)
{
    static uint8_t counter = 0, last_state = 0;

    if (last_state == 1) {
        if (_button == 1) ++counter;

        if (counter >= 12) {
            if (mode == NAV) mode = MANUAL;
            else
                mode = NAV;
            counter = 0;
        }
    } else {
        counter = 0;
    }

    last_state = static_cast<uint8_t>(_button);
}
