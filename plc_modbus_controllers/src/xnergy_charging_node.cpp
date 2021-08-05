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

bool start_charging(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if(xn_sensors.battery_volt >= 27.4){
        ROS_WARN("BATTERY IS FULLY CHARGED, CHARGE COMMAND REJECTED");
    }
    else{
        plc_modbus_node::ByteArray data;
        // initialising data variable with something
        data.name = "xnergy";
        data.data.push_back(xnergy_sensors::toggle); // On toggle to switch state
        data.data.push_back(xnergy_sensors::START_CHARGE); // Start charge
        coils_write.publish(data);
        ROS_INFO("CAUTION!!! Battery is being charged");
    }
        
    return true;
}

bool stop_charging(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    plc_modbus_node::ByteArray data;
    data.name = "xnergy";
    data.data.push_back(xnergy_sensors::toggle); // On toggle to switch state
    data.data.push_back(xnergy_sensors::STOP_CHARGE); // Stop charge
    coils_write.publish(data);
    ROS_INFO("Battery is not charging");
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
    ros::ServiceServer service_charge = nh.advertiseService("xnergy_node/start_charging", start_charging);
    ros::ServiceServer service_stop_charge = nh.advertiseService("xnergy_node/stop_charging", stop_charging);

    ROS_INFO("Xnergy node is ready");
    ros::spin();

    return 0;
}