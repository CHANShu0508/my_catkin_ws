#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "turtlesim/Pose.h"

std::string turtle_name;

void PoseCallback(const turtlesim::PoseConstPtr& _msg);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "my_tf2_broadcaster");
    ros::NodeHandle private_node("~");

    if (!private_node.hasParam("turtle")) {  // for the rosrun way
        if (argc != 2) {
            ROS_ERROR("Need turtle name as argument!");
            return 1;
        }
        turtle_name = argv[1];
    } else {   // for the '.launch' way
        private_node.getParam("turtle", turtle_name);
    }

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &PoseCallback);

    ros::spin();

    return 0;
}

void PoseCallback(const turtlesim::PoseConstPtr& _msg) 
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = turtle_name;
    transform_stamped.transform.translation.x = _msg->x;
    transform_stamped.transform.translation.y = _msg->y;
    transform_stamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, _msg->theta);
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    br.sendTransform(transform_stamped);
}
