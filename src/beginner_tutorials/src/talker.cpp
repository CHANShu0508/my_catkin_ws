#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char *argv[])
{
    // Init the node, receive param from console to rename the node
    ros::init(argc, argv, "talker");
    // Create a handle for this node in this thread
    ros::NodeHandle n;
    /** Tell the master node that this node will publish:
     *    - std_msgs::String type message
     *    - On a topic called "chatter"
     *    - The message will cache for most 1000
     **/
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    // ros::Rate object will record how long it takes from last time sleep() is called
    // to help you do something in loop, this example it is 10Hz
    ros::Rate loop_rate(2);

    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "Hello ROS! " << count;
        msg.data = ss.str();

        ROS_WARN("%s", msg.data.c_str());
        printf("%s\n", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
