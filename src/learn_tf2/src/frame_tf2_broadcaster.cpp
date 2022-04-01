#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "my_tf2_broadcaster");
    ros::NodeHandle node_handle;

    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.frame_id = "turtle1";
    transform_stamped.child_frame_id = "carrot1";
    transform_stamped.transform.translation.x = -1.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    ros::Rate rate(10.0);
    while (node_handle.ok()) {
        transform_stamped.header.stamp = ros::Time::now();
        tfb.sendTransform(transform_stamped);
        rate.sleep();
        printf("Sending\n");
    }

    return 0;
}
