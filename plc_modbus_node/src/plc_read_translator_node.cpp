// README: This node subscribes to modbus/coils&regs read
// and publishes to the forklift_sensors node, roboteq_sensors node, main_controller node
// and xnergy node

#include <ros/ros.h>
#include <plc_modbus_node/MultiUInt16Array.h>
#include <plc_modbus_node/MultiByteArray.h>
#include <modbus/modbus.h>
#include "plc_modbus_node/roboteq_sensors.h"
#include "plc_modbus_node/forklift_sensors.h"
#include "plc_modbus_node/main_controller.h"
#include "plc_modbus_node/xnergy_sensors.h"

#include <bitset>

plc_modbus_node::roboteq_sensors rob_sensors;
plc_modbus_node::forklift_sensors fl_sensors;
plc_modbus_node::xnergy_sensors xn_sensors;
plc_modbus_node::main_controller main_controller;

//Initialising regs/coils variables for the required type needed
// Main
bool heartbeat(0), estop_status(0);

// RoboteQ
int32_t speed_left(0), speed_right(0), encoder_left(0), encoder_right(0);
float amps_left(0), amps_right(0), volts_batt(0);
std::string fault_flag("");
uint16_t refresh_rate(0), time_elapsed(0);

// Forklift
float angle(0);
uint16_t lift_cmd(0), ir_cmd(0), ir_dist_left(0), ir_dist_right(0);
bool mount_status(0), alignment(0), busy_status(0);

// xnergy
uint16_t xnergy_runtime_voltage(0), xnergy_runtime_current(0), rcu_temp(0), batt_output_current(0);
uint16_t battery_volt(0), error_code(0);

void reg_clbk(const plc_modbus_node::MultiUInt16Array::ConstPtr& regs_data) {
  // NOTE: regs_data has uint16_t datatype, some need conversion into respective datatypes

  for (int i = 0; i < regs_data->arrays.size(); ++i) {
    // RoboteQ
    if (regs_data->arrays[i].name.compare("roboteq") == 0) {
      speed_left = (((uint16_t)regs_data->arrays[i].data.at(1) << 16)| (uint16_t)regs_data->arrays[i].data.at(0));
      speed_right = (((uint16_t)regs_data->arrays[i].data.at(3) << 16)| (uint16_t)regs_data->arrays[i].data.at(2));
      encoder_left = (((uint16_t)regs_data->arrays[i].data.at(5) << 16)| (uint16_t)regs_data->arrays[i].data.at(4));
      encoder_right = (((uint16_t)regs_data->arrays[i].data.at(7) << 16)| (uint16_t)regs_data->arrays[i].data.at(6));
      amps_left = (float)regs_data->arrays[i].data.at(8) / 10.0f;
      amps_right = (float)regs_data->arrays[i].data.at(9) / 10.0f;
      volts_batt = (float)regs_data->arrays[i].data.at(10) / 10.0f;
      int fault_flags = regs_data->arrays[i].data.at(11);
      fault_flag = std::bitset<16>(fault_flags).to_string();
      refresh_rate = regs_data->arrays[i].data.at(14);
      time_elapsed = regs_data->arrays[i].data.at(15);
    }
    // Forklift
    else if (regs_data->arrays[i].name.compare("forklift") == 0) {
      lift_cmd = regs_data->arrays[i].data.at(0);
      ir_cmd = regs_data->arrays[i].data.at(1);
      ir_dist_left = regs_data->arrays[i].data.at(2);
      ir_dist_right = regs_data->arrays[i].data.at(3);

      int angle_temp = (regs_data->arrays[i].data.at(5) << 16) | regs_data->arrays[i].data.at(4);
      memcpy(&angle, &angle_temp, sizeof(float));
    }
    else if (regs_data->arrays[i].name.compare("xnergy") == 0) {
      xnergy_runtime_voltage = regs_data->arrays[i].data.at(0);
      xnergy_runtime_current = regs_data->arrays[i].data.at(1);
      rcu_temp = regs_data->arrays[i].data.at(2);
      batt_output_current = regs_data->arrays[i].data.at(3);
      battery_volt = regs_data->arrays[i].data.at(4);
      error_code = (((uint16_t)regs_data->arrays[i].data.at(6) << 16)| (uint16_t)regs_data->arrays[i].data.at(5));

    }
  }
}

void coil_clbk(const plc_modbus_node::MultiByteArray::ConstPtr &coils_data) {
  // NOTE: data has bytes(char) datatype, need conversion into boolean datatype

  for (int i = 0; i < coils_data->arrays.size(); ++i) {
    // RoboteQ
    if (coils_data->arrays[i].name.compare("main") == 0) {
      heartbeat = (coils_data->arrays[i].data.at(0))!=0; 
      estop_status = (coils_data->arrays[i].data.at(1))!=0;
    }
    else if (coils_data->arrays[i].name.compare("forklift") == 0) {
      mount_status = (coils_data->arrays[i].data.at(0))!=0;
      alignment = (coils_data->arrays[i].data.at(1))!=0;
      busy_status = (coils_data->arrays[i].data.at(2))!=0;
    }
  }
   
}

void initialiseMessage(){

  // Main
  main_controller.heartbeat     = heartbeat;
  main_controller.estop_status  = estop_status ;

  // RoboteQ
  rob_sensors.speed_left    = speed_left ;
  rob_sensors.speed_right   = speed_right ;
  rob_sensors.encoder_left  = encoder_left ;
  rob_sensors.encoder_right = encoder_right;
  rob_sensors.amps_left    = amps_left;
  rob_sensors.amps_right   = amps_right  ;
  rob_sensors.volts_batt    = volts_batt ;
  rob_sensors.fault_flag    = fault_flag ;
  rob_sensors.refresh_rate  = refresh_rate;
  rob_sensors.time_elapsed  = time_elapsed ;
  
  // Forklift
  fl_sensors.lift_cmd       = lift_cmd;
  fl_sensors.ir_cmd         = ir_cmd;
  fl_sensors.ir_dist_left   = ir_dist_left;
  fl_sensors.ir_dist_right  = ir_dist_right;
  fl_sensors.angle          = angle;
  fl_sensors.mount_status   = mount_status ;
  fl_sensors.alignment     = alignment  ;
  fl_sensors.busy_status  = busy_status;

  // xnergy
  xn_sensors.xnergy_runtime_voltage = xnergy_runtime_voltage ;
  xn_sensors.xnergy_runtime_current = xnergy_runtime_current;
  xn_sensors.rcu_temp = rcu_temp ;
  xn_sensors.batt_output_current = batt_output_current ;
  xn_sensors.battery_volt = battery_volt ;
  xn_sensors.error_code = error_code ;
             
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "read_translator_node");

  ros::NodeHandle n;
  
  // Subscribe to the respective topics
  ros::Subscriber sub_reg = n.subscribe("modbus/regs_read", 100, reg_clbk);
  ros::Subscriber sub_coil = n.subscribe("modbus/coils_read", 100, coil_clbk);
  
  // Publish to the respective topics
  ros::Publisher pub_roboteq_sensors = n.advertise<plc_modbus_node::roboteq_sensors>("/modbus/roboteq_sensors", 100);
  ros::Publisher pub_forklift_sensors = n.advertise<plc_modbus_node::forklift_sensors>("/modbus/forklift_sensors", 100);
  ros::Publisher pub_xnergy_sensors = n.advertise<plc_modbus_node::xnergy_sensors>("/modbus/xnergy_sensors", 100);
  ros::Publisher pub_main_controller = n.advertise<plc_modbus_node::main_controller>("/modbus/main_controller", 100);
  
  ros::Rate loop_rate(10);

  while(ros::ok()){
    
    initialiseMessage(); //function to assign the respective values for each message
      
    pub_roboteq_sensors.publish(rob_sensors);
    pub_forklift_sensors.publish(fl_sensors);
    pub_xnergy_sensors.publish(xn_sensors);
    pub_main_controller.publish(main_controller);

    ros::spinOnce();
    loop_rate.sleep();
    
  } 

}
