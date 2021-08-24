#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <plc_modbus_node/UInt16Array.h>
#include <plc_modbus_node/ByteArray.h>
#include <plc_modbus_node/forklift_sensors.h>

#include <plc_modbus_controllers/MoveForkliftAction.h>
#include <actionlib/server/simple_action_server.h>

using plc_modbus_node::forklift_sensors;

typedef actionlib::SimpleActionServer<plc_modbus_controllers::MoveForkliftAction> Server;


// Forklift sensor variables
forklift_sensors fl_sensors;

ros::Subscriber sensors_read;
ros::Publisher regs_write;

// Action variables
plc_modbus_controllers::MoveForkliftFeedback actionFeedback_;
plc_modbus_controllers::MoveForkliftResult actionResult_;


// ROS Service
bool forklift_up(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if (fl_sensors.lift_cmd != 0 || fl_sensors.busy_status != false){
        // Forklift is completing previous command, don't issue command
        ROS_WARN("FORKLIFT IS BUSY, UP COMMAND REJECTED");        
    }
    else if (fl_sensors.mount_status == true){
        // Forklift is UP, don't issue command
        ROS_WARN("FORKLIFT IS UP, UP COMMAND REJECTED");
    }
    else{
        // publish the command to /modbus/regs_write once
        plc_modbus_node::UInt16Array data;
        data.name = "forklift";
        data.data.push_back(forklift_sensors::CMD_LIFT_UP); // Lift Motor Command
        data.data.push_back(forklift_sensors::CMD_NO_IR); // IR Command

        regs_write.publish(data);
    }

    return true;
}

bool forklift_down(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if (fl_sensors.lift_cmd != 0 || fl_sensors.busy_status != false){
        // Forklift is completing previous command, don't issue command
        ROS_WARN("FORKLIFT IS BUSY, DOWN COMMAND REJECTED");        
    }
    else if (fl_sensors.mount_status == false){
        // Forklift is DOWN, don't issue command
        ROS_WARN("FORKLIFT IS DOWN, DOWN COMMAND REJECTED");
    }
    else{
        // publish the command to /modbus/regs_write once
        plc_modbus_node::UInt16Array data;
        data.name = "forklift";
        data.data.push_back(forklift_sensors::CMD_LIFT_DOWN); // Lift Motor Command
        data.data.push_back(forklift_sensors::CMD_NO_IR); // IR Command

        regs_write.publish(data);
    }

    return true;
}

bool forklift_ir(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    // publish the command to /modbus/regs_write once
    plc_modbus_node::UInt16Array data;
    data.name = "forklift";
    data.data.push_back(forklift_sensors::CMD_NO_LIFT); // Lift Motor Command
    data.data.push_back(forklift_sensors::CMD_IR); // IR Command

    regs_write.publish(data);

    return true;
}

// ROS Action
void execute(const plc_modbus_controllers::MoveForkliftGoalConstPtr& goal, Server* as)
{
    // Check that the forklift is not currently in action
    if (fl_sensors.lift_cmd != 0 || fl_sensors.busy_status != false){
        // Forklift is completing previous command, don't issue command
        ROS_WARN("FORKLIFT IS BUSY, DOWN COMMAND REJECTED");

        actionResult_.success = false;
        as->setSucceeded(actionResult_);
        return;
    }

    // Check that the goal is valid
    if (goal->move_up == false && fl_sensors.mount_status == false) {  // goal is to move down
        // Forklift is DOWN, don't issue command
        ROS_WARN("FORKLIFT IS DOWN, DOWN COMMAND REJECTED");

        actionResult_.success = false;
        as->setSucceeded(actionResult_);
        return;
    }
    else if (goal->move_up == true && fl_sensors.mount_status == true) {  // goal is to move up
        // Forklift is UP, don't issue command
        ROS_WARN("FORKLIFT IS UP, UP COMMAND REJECTED");

        actionResult_.success = false;
        as->setSucceeded(actionResult_);
        return;
    }

    // Publish info to the console for the user
    ROS_INFO("Executing Forklift Move %s", goal->move_up ? "Up" : "Down");

    // Publish the lift command to the modbus to start the action
    plc_modbus_node::UInt16Array data;
    data.name = "forklift";
    data.data.push_back(goal->move_up ? forklift_sensors::CMD_LIFT_UP : forklift_sensors::CMD_LIFT_DOWN); // Lift Motor Command
    data.data.push_back(forklift_sensors::CMD_NO_IR); // IR Command
    regs_write.publish(data);

    // Wait a bit for the sensor readings to update
    ros::Duration(0.5).sleep();

    while (fl_sensors.busy_status) {  // still executing the action
        actionFeedback_.busy_status = fl_sensors.busy_status;
        as->publishFeedback(actionFeedback_);
    }

    if (goal->move_up)  // move up
        actionResult_.success = fl_sensors.mount_status;  // true if mounted trolley successfully
    else  // move down
        actionResult_.success = !fl_sensors.mount_status;  // true if not mounted trolley
    as->setSucceeded(actionResult_);
}

void sensors_callback(const forklift_sensors::ConstPtr& data){
    fl_sensors = *data;
    // ROS_INFO_STREAM(fl_sensors);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forklift_node");
    ros::NodeHandle nh;

    // subscribe to forklift sensors
    sensors_read = nh.subscribe<forklift_sensors>("modbus/forklift_sensors", 100, sensors_callback);
    
    regs_write = nh.advertise<plc_modbus_node::UInt16Array>("modbus/regs_write", 100);

    // advertise forklift command services
    ros::ServiceServer service_up = nh.advertiseService("forklift_node/forklift_up", forklift_up);
    ros::ServiceServer service_down = nh.advertiseService("forklift_node/forklift_down", forklift_down);
    ros::ServiceServer service_ir = nh.advertiseService("forklift_node/forklift_ir", forklift_ir);

    // ROS action
    Server server(nh, "move_forklift", boost::bind(&execute, _1, &server), false);
    server.start();

    ROS_INFO("Forklift node is ready");
    ros::spin();

    return 0;
}