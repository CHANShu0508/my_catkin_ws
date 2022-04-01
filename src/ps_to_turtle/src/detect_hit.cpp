#include "ros/ros.h"
#include "ds4_driver/Feedback.h"
#include "turtlesim/Pose.h"

class DetectHit {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

public:
    void PoseSubCallback(const turtlesim::Pose::ConstPtr& msg);

    DetectHit() {
        pub_ = nh_.advertise<ds4_driver::Feedback>("set_feedback", 100);
        sub_ = nh_.subscribe("turtle1/pose", 100, &DetectHit::PoseSubCallback, this);
    }

    turtlesim::Pose turtle_pose;
    ds4_driver::Feedback feedback;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "detect_hit");

    DetectHit detect;
    ros::spin();

    return 0;
}

void DetectHit::PoseSubCallback(const turtlesim::Pose::ConstPtr& msg)
{
    bool is_left = (msg->x - 0.0 < 1e-4);
    bool is_right = (msg->x > 11.0);
    bool is_up = (msg->y - 0.0 < 1e-4);
    bool is_down = (msg->y > 11.0);

    int sum = (int)is_left + (int)is_right + (int)is_up + (int)is_down;

    if (sum >= 1) {
        feedback.set_rumble = true;
        feedback.rumble_big = 0.8;
        feedback.rumble_small = 0.8;
    } else {
        feedback.set_rumble = false;
    }

    pub_.publish(feedback);
}
