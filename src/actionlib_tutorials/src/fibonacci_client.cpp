#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "actionlib_tutorials/FibonacciAction.h"
#include <stdlib.h>

int main(int argc, char *argv[])
{
    int receive_order;
    actionlib_tutorials::FibonacciResult result;
    ros::init(argc, argv, "fibonacci_client");

    if (argc != 2) {
        ROS_WARN("Usage: fibonacci_client order");
        return 1;
    } else receive_order = atoi(argv[1]);
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); // this will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    actionlib_tutorials::FibonacciGoal goal;
    goal.order = receive_order;
    ac.sendGoal(goal);

    // wait for the action to return
    bool finishied_before_timeout = ac.waitForResult(ros::Duration((double)receive_order + 1.0));

    if (finishied_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        result = *ac.getResult();
        for (int i = 0; i < receive_order; i++) {
            printf("%d\n", result.sequence[i]);
        }
    } else {
        ROS_INFO("Action did not finish before the time out.");
    }

    return 0;
}
