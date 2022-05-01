#include "fork_control.h"
#include <std_msgs/Float64.h>

#define MAX_POSITION 1.6

// --------------------------------------------- MAIN FUNCTON ---------------------------------------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fork_control");

    ForkCtrl fork;

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        fork.PublishData();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
// --------------------------------------------------------------------------------------------------------

ForkCtrl::ForkCtrl()
{
    this->fork_position = 0.0;
    this->fork_cmd_val = 0.0;
    this->sub_ = nh_.subscribe("forklift/fork_cmd", 20, &ForkCtrl::CmdCallback, this);
    this->cmd_pub_ = nh_.advertise<std_msgs::Float64>("forklift_controllers/fork_controller/command", 20);
}

void ForkCtrl::CmdCallback(const std_msgs::Float64MultiArrayConstPtr &_ptr)
{
    bool is_left_zero = (_ptr->data[0] - 0.0 < 1e-4);
    bool is_right_zero = (_ptr->data[1] - 0.0 < 1e-4);

    if (is_left_zero && is_right_zero) {
        this->fork_cmd_val = 0.0;
    } else if (!is_left_zero && !is_right_zero) {
        this->fork_cmd_val = 0.0;
    } else {
        if (!is_left_zero) this->fork_cmd_val = 0.01 * _ptr->data[0];// Publish rate is 100, so 1m per second
        if (!is_right_zero) this->fork_cmd_val = -0.01 * _ptr->data[1];
    }
}

void ForkCtrl::PublishData()
{
    std_msgs::Float64 send_msg;

    this->fork_position += this->fork_cmd_val;
    if (this->fork_position > MAX_POSITION)
        this->fork_position = MAX_POSITION;
    else if (this->fork_position < 0.0)
        this->fork_position = 0.0;

    send_msg.data = this->fork_position;

    this->cmd_pub_.publish(send_msg);
}
