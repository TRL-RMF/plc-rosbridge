// this subscribes to the coil/register write
// this also publishes to roboteq_sensors/speed


#include "ros/ros.h"
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include "plc_modbus_node/roboteq_sensors.h"

plc_modbus_node::roboteq_sensors sensors;

void reg_clbk(const std_msgs::UInt16MultiArray::ConstPtr &regs_data) {
  for (int i = 0; i < regs_data->data.size(); i++) {
    ROS_INFO("regs %d: %u", i, regs_data->data.at(i));
  }
  // data conversion and writing to sensor msg
  sensors.encoder_right = (((uint16_t)regs_data->data.at(0) << 16)| (uint16_t)regs_data->data.at(1));
}

void coil_clbk(const std_msgs::ByteMultiArray::ConstPtr &coils_data) {
  for (int i = 0; i < coils_data->data.size(); i++) {
    ROS_INFO("coils %d: %u", i, coils_data->data.at(i));
  }
}

int main(int argc, char **argv)
{

  // ROS objects
  ros::init(argc, argv, "my_subscriber");

  ros::NodeHandle n;
  ros::Subscriber sub_reg = n.subscribe("modbus/regs_read", 100, reg_clbk);
  ros::Subscriber sub_coil = n.subscribe("modbus/coils_read", 100, coil_clbk);
  ros::Publisher pub_roboteq_sensors = n.advertise<plc_modbus_node::roboteq_sensors>("roboteq_sensors/encoders", 100);
  ros::Publisher pub_roboteq_sensors = n.advertise<plc_modbus_node::roboteq_sensors>("roboteq_sensors/encoders", 100);
  
  ros::Rate loop_rate(10);

  while(ros::ok()){
    pub_roboteq_sensors.publish(sensors);
    
    ros::spinOnce();
    loop_rate.sleep();
    
  } 

}
