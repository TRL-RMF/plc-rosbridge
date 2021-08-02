#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <plc_modbus_node/UInt16Array.h>
#include <plc_modbus_node/ByteArray.h>
#include <plc_modbus_node/forklift_sensors.h>

using plc_modbus_node::forklift_sensors;

// Forklift sensor variables
forklift_sensors fl_sensors;

ros::Subscriber sensors_read;
ros::Publisher regs_write;

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

    ROS_INFO("Forklift node is ready");
    ros::spin();

    return 0;
}