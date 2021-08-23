#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <plc_modbus_controllers/MoveForkliftAction.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_forklift_actionclient");

    // Create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<plc_modbus_controllers::MoveForkliftAction> ac("move_forklift", true);

    ROS_INFO("Waiting for action server to start.");

    // Wait for the action server to start
    ac.waitForServer();  // will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // create action goal
    plc_modbus_controllers::MoveForkliftGoal goal;
    // get move up or down from ros params
    ros::NodeHandle n;
    bool move_up;
    n.getParam("move_up", move_up);
    ROS_INFO("Action: Move %s", move_up ? "Up" : "Down");
    goal.move_up = move_up;

    // send the goal
    ac.sendGoal(goal);

    // Wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action successful: %s",state.toString().c_str());

        //manipulator::MoveForkliftResultConstPtr result = ac.getResult();
        ROS_INFO("Result (i.e. whether the forklift action was successful): %s", ac.getResult()->success ? "True" : "False");
    }
    else {
        ROS_INFO("Action did not finish before the time out.");
    }

    return 0;
}