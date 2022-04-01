#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

bool AddResponse(beginner_tutorials::AddTwoInts::Request  &_req,
                 beginner_tutorials::AddTwoInts::Response &_res)
{
    _res.sum = _req.a + _req.b;
    ROS_INFO("Request: x = %ld, y = %ld", (long int)_req.a, (long int)_req.b);
    ROS_INFO("Sending back response: [%ld]", (long int)_res.sum);
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle node_handle;

    ros::ServiceServer server = node_handle.advertiseService("add_two_ints", AddResponse);
    ROS_INFO("Ready to add two ints!");
    ros::spin();

    return 0;
}
