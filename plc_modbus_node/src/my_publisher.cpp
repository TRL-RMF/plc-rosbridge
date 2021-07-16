// subscribes to topic: motor_control
// publishes to reg_write

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <sstream>
#include "plc_modbus_node/roboteq_speed.h"

plc_modbus_node::roboteq_sensors speed;

void reg_clbk(const std_msgs::UInt16MultiArray::ConstPtr &regs_data) {
  for (int i = 0; i < regs_data->data.size(); i++) {
    ROS_INFO("regs %d: %u", i, regs_data->data.at(i));
  }
  // data conversion and writing to speed msg
  speed.right_speed = (((uint16_t)regs_data->data.at(0) << 16)| (uint16_t)regs_data->data.at(1));
    
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle node;

  
  // subsciber node that subscribes from motor_control topic
  ros::Subscriber regs_read;
  regs_read = node.subscribe<std_msgs::UInt16MultiArray>("motor_control", 100, reg_clbk);
  
  // publisher node that publishes to modbus/reg_write topic
  ros::Publisher regs_write;
  regs_write= node.advertise<std_msgs::UInt16MultiArray>("modbus/regs_write", 100);


  ros::Rate loop_rate(10);

  
  while(ros::ok()){
    regs_write.publish(speed);
    ros::spinOnce();
    loop_rate.sleep();
    
  }   

  return 0;
}