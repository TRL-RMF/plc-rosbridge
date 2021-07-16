// subscribes to topic: motor_control
// publishes to reg_write

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <sstream>
#include "plc_modbus_node/roboteq_speed.h"

plc_modbus_node::roboteq_sensors speed;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle node;

  
  // subsciber node that subscribes from motor_control topic
  ros::Subscriber regs_read;
  regs_read = node.subscribe<std_msgs::UInt16MultiArray>("motor_control", 100);
  
  // publisher node that publishes to modbus/reg_write topic
  ros::Publisher regs_write;
  regs_write= node.advertise<std_msgs::UInt16MultiArray>("modbus/regs_write", 100);


  ros::Rate loop_rate(10);

  
  while(ros::ok()){
    regs_write.publish(speed);
    
    ros::spinOnce();
    loop_rate.sleep();
    
  }   



  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(msg);
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}