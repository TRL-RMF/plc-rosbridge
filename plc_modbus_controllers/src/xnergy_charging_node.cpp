#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <plc_modbus_node/UInt16Array.h>
#include <plc_modbus_node/ByteArray.h>
#include <plc_modbus_node/xnergy_sensors.h>

using plc_modbus_node::xnergy_sensors;

// Xnergy sensor variables
xnergy_sensors xn_sensors;

ros::Subscriber sensors_read;
ros::Publisher coils_write;

bool charge_cmds(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
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

void sensors_callback(const plc_modbus_node::xnergy_sensors::ConstPtr& data){
    xn_sensors = *data;
    // ROS_INFO_STREAM(xn_sensors);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xnergy_node");
    ros::NodeHandle nh;

    // subscribe to xnergy sensors
    sensors_read = nh.subscribe("/modbus/xnergy_sensors", 100, sensors_callback);
    
    //publish to col write topics
    coils_write = nh.advertise<plc_modbus_node::ByteArray>("modbus/coils_write", 100);

    // advertise xnergy command services
    ros::ServiceServer service_up = nh.advertiseService("xnergy_node/charging_services", charge_cmds);

    ROS_INFO("Xnergy node is ready");
    ros::spin();

    return 0;
}