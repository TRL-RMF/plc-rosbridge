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
ros::Publisher regs_write;

void sensors_callback(const forklift_sensors::ConstPtr& data){
    xn_sensors = *data;
    // ROS_INFO_STREAM(xn_sensors);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xnergy_node");
    ros::NodeHandle nh;

    // subscribe to xnergy sensors
    sensors_read = nh.subscribe("/modbus/xnergy_sensors", 100, sensors_callback);
    
    regs_write = nh.advertise<plc_modbus_node::UInt16Array>("modbus/regs_write", 100);

    // advertise xnergy command services
    /*ros::ServiceServer service_up = nh.advertiseService("forklift_node/forklift_up", forklift_up);
    ros::ServiceServer service_down = nh.advertiseService("forklift_node/forklift_down", forklift_down);
    ros::ServiceServer service_ir = nh.advertiseService("forklift_node/forklift_ir", forklift_ir);*/

    ROS_INFO("Xnergy node is ready");
    ros::spin();

    return 0;
}