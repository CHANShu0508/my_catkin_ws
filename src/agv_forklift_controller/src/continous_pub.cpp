#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char *argv[])
{
    std::string true_ = "true", false_ = "false";
    ros::init(argc, argv, "continous_pub");
    ros::NodeHandle nh;
    if (argc != 5) {
        ROS_WARN("Usage: continous_pub [topic_name] [rate] [switch] [data_value(double)]\n"
        "\tFor switch: true for multiFloat64 and false for Float64. And we send only four");
        return 1;
    }

    ros::Rate loop_rate(atoi(argv[2]));

    if (argv[3] == false_) {
        ros::Publisher pub = nh.advertise<std_msgs::Float64>(argv[1], 10);

        while (ros::ok()) {
            std_msgs::Float64 msg;

            msg.data = atof(argv[4]);
            pub.publish(msg);

            loop_rate.sleep();
        }
    } else if (argv[3] == true_) {
        ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>(argv[1], 10);

        while (ros::ok()) {
            std_msgs::Float64MultiArray msg_multi;

            msg_multi.data.push_back(atof(argv[4]));
            msg_multi.data.push_back(atof(argv[4]));
            msg_multi.data.push_back(atof(argv[4]));
            msg_multi.data.push_back(atof(argv[4]));

            pub.publish(msg_multi);

            loop_rate.sleep();
        }
    } else {
        ROS_WARN("Bad value for switch!");
        return 1;
    }

    return 0;
}
