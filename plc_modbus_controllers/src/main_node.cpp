#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <plc_modbus_node/UInt16Array.h>
#include <plc_modbus_node/ByteArray.h>
#include <plc_modbus_node/main_controller.h>

// Forklift sensor variables
plc_modbus_node::main_controller main_controller;

ros::Subscriber main_controller_read;
ros::Publisher pub_main_controller;

// data to publish
plc_modbus_node::ByteArray byteData;

bool first = true;

void main_callback(const plc_modbus_node::main_controller::ConstPtr& data){
    main_controller = *data;
    // ROS_INFO_STREAM(fl_sensors);

    first = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;

    // subscribe to main controller
    main_controller_read = nh.subscribe<plc_modbus_node::main_controller>("modbus/main_controller", 100, main_callback);
    // publish main controller information
    pub_main_controller = nh.advertise<plc_modbus_node::ByteArray>("modbus/coils_write", 100);
    byteData.name = "main";

    ROS_INFO("Main node is ready");

    ros::Rate loop_rate(1);
    while(ros::ok()){
    
        if (!first) {
            byteData.data.clear();
            byteData.data.push_back(1);
            byteData.data.push_back(main_controller.estop_status);
            pub_main_controller.publish(byteData);
        }

        ros::spinOnce();
        loop_rate.sleep();
        
    } 

    return 0;
}