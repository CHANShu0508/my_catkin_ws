#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"

// TODO: The class need a way to automatically change string to double

int main(int argc, char *argv[])
{
    std::string new_spawned_name; // the new frame name
    float x_coord, y_coord;

    ros::init(argc, argv, "my_tf2_listener");
    ros::NodeHandle private_nh("~pri_nh");

    bool if_get_name = private_nh.getParam("spawned_name", new_spawned_name);
    bool if_get_x = private_nh.getParam("x_cord", x_coord);
    bool if_get_y = private_nh.getParam("y_cord", y_coord);

    // debug
    // if (!if_get_name) { ROS_WARN("Do not get the name!!"); }

    // Do the control work
    ros::NodeHandle listener_nh;
    // Spawn a turtle
    ros::service::waitForService("spawn");
    ros::ServiceClient spawner = listener_nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn turtle;
    turtle.request.x = x_coord;
    turtle.request.y = y_coord;
    turtle.request.theta = 0;
    turtle.request.name = new_spawned_name;
    bool is_spawn_success =  spawner.call(turtle);

    // debug
    // if (!is_spawn_success) { ROS_WARN("Spawn failed!"); }

    ros::Publisher turtle_vel = listener_nh.advertise<geometry_msgs::Twist>(new_spawned_name+"/cmd_vel", 10);
    
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::Rate rate(10.0);
    while (listener_nh.ok()) {
        geometry_msgs::TransformStamped transform_stamped;
        try {
            // For following the track 4 seconds in the past
            // ros::Time now = ros::Time::now();
            // ros::Time past = now - ros::Duration(4.0);
            // transform_stamped = tf_buffer.lookupTransform(new_spawned_name, now, "carrot1", past, "world", ros::Duration(1.0));
            transform_stamped = tf_buffer.lookupTransform(new_spawned_name, "turtle1", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 4.0 * atan2(transform_stamped.transform.translation.y, 
                                        transform_stamped.transform.translation.x);
        vel_msg.linear.x = 0.5 * sqrt(pow(transform_stamped.transform.translation.x, 2) + 
                                      pow(transform_stamped.transform.translation.y, 2));
        turtle_vel.publish(vel_msg);

        rate.sleep();
    }

    return 0;
}
