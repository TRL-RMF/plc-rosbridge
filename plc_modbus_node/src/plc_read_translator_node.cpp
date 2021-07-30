// README: This node subscribes to modbus/coils&regs read
// and plublishes to the forklift_sensors node and the roboteq_sensors node 


#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <modbus/modbus.h>
#include "plc_modbus_node/roboteq_msg.h"
#include "plc_modbus_node/forklift_msg.h"

plc_modbus_node::roboteq_sensors rob_sensors;
plc_modbus_node::forklift_sensors fl_sensors;

//Initialising regs/coils variables for the required type needed
uint32_t speed_left,speed_right,encoder_left,encoder_right,angle;
int16_t amps_left,amps_right;
uint16_t volts_batt,refresh_rate,time_elapsed,lift_cmd,ir_cmd,ir_dist_left,ir_dist_right;
bool heartbeat,estop_status,mount_status,alignment;

void reg_clbk(const std_msgs::UInt16MultiArray::ConstPtr& regs_data) {
   
  // regs_data has uint16_t datatype, some need conversion into respective datatypes
  speed_left = (((uint16_t)regs_data->data.at(0) << 16)| (uint16_t)regs_data->data.at(1));
  speed_right = (((uint16_t)regs_data->data.at(2) << 16)| (uint16_t)regs_data->data.at(3));
  encoder_left = (((uint16_t)regs_data->data.at(4) << 16)| (uint16_t)regs_data->data.at(5));
  encoder_right = (((uint16_t)regs_data->data.at(6) << 16)| (uint16_t)regs_data->data.at(7));
  amps_left = regs_data->data.at(9);
  amps_right = regs_data->data.at(11);
  volts_batt = regs_data->data.at(13);
  refresh_rate = regs_data->data.at(14);
  time_elapsed = regs_data->data.at(15); 
  
  lift_cmd = regs_data->data.at(16);
  ir_cmd = regs_data->data.at(17);
  ir_dist_left = regs_data->data.at(18);
  ir_dist_right = regs_data->data.at(19);
  angle = (((uint16_t)regs_data->data.at(20) << 16)| (uint16_t)regs_data->data.at(21));

}

void coil_clbk(const std_msgs::ByteMultiArray::ConstPtr &coils_data) {

  // data has bytes(char) datatype, need conversion into boolean datatype  
  heartbeat = (coils_data->data.at(0))!=0; 
  estop_status = (coils_data->data.at(1))!=0; 
  mount_status = (coils_data->data.at(2))!=0;
  alignment = (coils_data->data.at(3))!=0;  
   
}

void initialiseMessage(){

    rob_sensors.speed_left_msg    = speed_left ;
    rob_sensors.speed_right_msg   = speed_right ;
    rob_sensors.encoder_left_msg  = encoder_left ;
    rob_sensors.encoder_right_msg = encoder_right;
    rob_sensors.amps_left_msg     = amps_left;
    rob_sensors.amps_right_msg    = amps_right  ;
    rob_sensors.volts_batt_msg    = volts_batt ;
    rob_sensors.refresh_rate_msg  = refresh_rate;
    rob_sensors.time_elapsed_msg  = time_elapsed ;
    rob_sensors.heartbeat_msg     = heartbeat;
    rob_sensors.estop_status_msg  = estop_status ;
    
    fl_sensors.lift_cmd_msg       = lift_cmd;
    fl_sensors.ir_cmd_msg         = ir_cmd;
    fl_sensors.ir_dist_left_msg   = ir_dist_left;
    fl_sensors.ir_dist_right_msg  = ir_dist_right;
    fl_sensors.angle_msg          = angle;
    fl_sensors.mount_status_msg   = mount_status ;
    fl_sensors.alignement_msg     = alignment  ;
           
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "read_translator_node");

  ros::NodeHandle n;
  
  // Subscribe to the respective topics
  ros::Subscriber sub_reg = n.subscribe("modbus/regs_read", 100, reg_clbk);
  ros::Subscriber sub_coil = n.subscribe("modbus/coils_read", 100, coil_clbk);
  
  // Publish to the respective topics
  ros::Publisher pub_roboteq_sensors = n.advertise<plc_modbus_node::roboteq_sensors>("roboteq_sensors_topic", 100);
  ros::Publisher pub_forklift_sensors = n.advertise<plc_modbus_node::forklift_sensors>("forklift_sensors_topic", 100);
  
  ros::Rate loop_rate(10);

  while(ros::ok()){
    
    initialiseMessage(); //function to assign the respective values for each message
      
    pub_roboteq_sensors.publish(rob_sensors);
    pub_forklift_sensors.publish(fl_sensors);

    std_msgs::UInt16MultiArray.data.clear();
    std_msgs::ByteMultiArray.data.clear();

    ros::spinOnce();
    loop_rate.sleep();
    
  } 

}
