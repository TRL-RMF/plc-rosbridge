#include <ros/ros.h>
#include <ros/console.h>
#include <string>

#include <plc_modbus_node/UInt16Array.h>
#include <plc_modbus_node/roboteq_sensors.h>

using plc_modbus_node::roboteq_sensors;

#define DELTAT(_nowtime,_thentime) ((_thentime>_nowtime)?((0xffffffff-_thentime)+_nowtime):(_nowtime-_thentime))


//
// cmd_vel subscriber
//

// Define following to enable cmdvel debug output
#define _CMDVEL_DEBUG

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


//
// odom publisher
//

// Define following to enable odom debug output
#define _ODOM_DEBUG

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>


uint32_t millis()
{
	ros::WallTime walltime = ros::WallTime::now();
//	return (uint32_t)((walltime._sec*1000 + walltime.nsec/1000000.0) + 0.5);
//	return (uint32_t)(walltime.toNSec()/1000000.0+0.5);
	return (uint32_t)(walltime.toNSec()/1000000);
}

class MainNode
{

public:
  MainNode();

public:

  //
  // cmd_vel subscriber
  //
  void cmdvel_callback( const geometry_msgs::Twist& twist_msg);
  void cmdvel_setup();
  // void cmdvel_loop();
  // void cmdvel_run();

  //
  // roboteq_sensor subscriber
  //
  void sensor_callback(const roboteq_sensors::ConstPtr& data);
  void sensor_setup();
  
  //
  // modbus registers publisher
  //
  void regs_write_setup();

  //
  // odom publisher
  //
  void odom_setup();
  // void odom_stream();
  void odom_loop();
  //void odom_hs_run();
  // void odom_ms_run();
  // void odom_ls_run();
  void odom_publish();

  int run();

protected:
  ros::NodeHandle nh;

  //serial::Serial controller;

  uint32_t starttime;
  // uint32_t hstimer;
  // uint32_t mstimer;
  // uint32_t lstimer;

  //
  // cmd_vel subscriber
  //
  ros::Subscriber cmdvel_sub;

  //
  // roboteq_sensors subscriber
  //
  ros::Subscriber sensor_sub;

  //
  // modbus registers publisher
  //
  ros::Publisher regs_write;

  // variables for roboteq sensors from modbus
  roboteq_sensors rb_sensors;
  uint32_t current_last_time;

  /* old
  float voltage;
  float current_right;
  float current_left;
  float energy;
  float temperature;
  uint32_t current_last_time;
  */

  //
  // odom publisher
  //
  geometry_msgs::TransformStamped tf_msg;
  tf::TransformBroadcaster odom_broadcaster;
  nav_msgs::Odometry odom_msg;
  ros::Publisher odom_pub;

  // buffer for reading encoder counts
  int odom_idx;
  char odom_buf[24];

  // toss out initial encoder readings
  char odom_encoder_toss;

  int32_t odom_encoder_left;
  int32_t odom_encoder_right;
  
  int32_t odom_rpm_left, odom_rpm_right;

  float odom_x;
  float odom_y;
  float odom_yaw;
  float odom_last_x;
  float odom_last_y;
  float odom_last_yaw;

  uint32_t odom_last_time;

  // settings
  bool pub_odom_tf;
  std::string odom_frame;
  std::string base_frame;
  std::string cmdvel_topic;
  std::string odom_topic;
  std::string regs_write_topic;
  std::string sensors_topic;
  std::string port;
  int baud;
  bool open_loop;
  double wheel_circumference;
  double track_width;
  int encoder_ppr;
  int encoder_cpr;
  double max_amps;
  int max_rpm;

};

MainNode::MainNode() : 
  starttime(0),
  // hstimer(0),
  // mstimer(0),
  odom_idx(0),
  odom_encoder_toss(5),
  odom_encoder_left(0),
  odom_encoder_right(0),
  odom_x(0.0),
  odom_y(0.0),
  odom_yaw(0.0),
  odom_last_x(0.0),
  odom_last_y(0.0),
  odom_last_yaw(0.0),
  odom_last_time(0),

  // voltage(0.0),
  // current_right(0.0),
  // current_left(0.0),
  // energy(0.0),
  // temperature(0.0),
  // current_last_time(0),
  
  pub_odom_tf(false),
  open_loop(false),
  // baud(115200),
  wheel_circumference(0),
  track_width(0),
  encoder_ppr(0),
  encoder_cpr(0),
  max_amps(0.0),
  max_rpm(0)
{


  // CBA Read local params (from launch file)
  ros::NodeHandle nhLocal("~");
  nhLocal.param("pub_odom_tf", pub_odom_tf, false);
  ROS_INFO_STREAM("pub_odom_tf: " << pub_odom_tf);
  nhLocal.param<std::string>("odom_frame", odom_frame, "odom");
  ROS_INFO_STREAM("odom_frame: " << odom_frame);
  nhLocal.param<std::string>("base_frame", base_frame, "base_link");
  ROS_INFO_STREAM("base_frame: " << base_frame);
  nhLocal.param<std::string>("cmdvel_topic", cmdvel_topic, "cmd_vel");
  ROS_INFO_STREAM("cmdvel_topic: " << cmdvel_topic);
  nhLocal.param<std::string>("odom_topic", odom_topic, "odom");
  ROS_INFO_STREAM("odom_topic: " << odom_topic);
  nhLocal.param<std::string>("regs_write_topic", regs_write_topic, "modbus/regs_write");
  ROS_INFO_STREAM("regs_write_topic: " << regs_write_topic);
  nhLocal.param<std::string>("sensors_topic", sensors_topic, "modbus/roboteq_sensors");
  ROS_INFO_STREAM("sensors_topic: " << sensors_topic);
  nhLocal.param("open_loop", open_loop, false);
  ROS_INFO_STREAM("open_loop: " << open_loop);
  nhLocal.param("wheel_circumference", wheel_circumference, 0.678);
  ROS_INFO_STREAM("wheel_circumference: " << wheel_circumference);
  nhLocal.param("track_width", track_width, 0.414);
  ROS_INFO_STREAM("track_width: " << track_width);
  nhLocal.param("encoder_ppr", encoder_ppr, 900);
  ROS_INFO_STREAM("encoder_ppr: " << encoder_ppr);
  nhLocal.param("encoder_cpr", encoder_cpr, 3600);
  ROS_INFO_STREAM("encoder_cpr: " << encoder_cpr);
  nhLocal.param("max_amps", max_amps, 5.0);
  ROS_INFO_STREAM("max_amps: " << max_amps);
  nhLocal.param("max_rpm", max_rpm, 100);
  ROS_INFO_STREAM("max_rpm: " << max_rpm);

}


//
// cmd_vel subscriber
//

void MainNode::cmdvel_callback( const geometry_msgs::Twist& twist_msg)
{

  // wheel speed (m/s)
  float right_speed = twist_msg.linear.x + track_width * twist_msg.angular.z / 2.0;
  float left_speed = twist_msg.linear.x - track_width * twist_msg.angular.z / 2.0;
  ROS_INFO_STREAM("cmdvel speed right: " << right_speed << " left: " << left_speed);

  int32_t right_cmd;
  int32_t left_cmd;

  if (open_loop)
  {
    // motor power (scale 0-1000)
    int32_t right_power = (float)right_speed / (float)wheel_circumference * 60.0 / max_rpm * 1000.0;
    int32_t left_power = left_speed / wheel_circumference * 60.0 / max_rpm * 1000.0;
    ROS_INFO_STREAM("cmdvel power right: " << right_power << " left: " << left_power);
    // right_cmd << "!G 1 " << right_power << "\r";
    // left_cmd << "!G 2 " << left_power << "\r";
    // std::cout << "open-loop: right_cmd: " << right_power << std::endl;
    // std::cout << "open-loop: left_cmd: " << left_power << std::endl;
    right_cmd = right_power;
    left_cmd = left_power;
    
  }
  else
  {
    // motor speed (rpm)
    int32_t right_rpm = right_speed / wheel_circumference * 60.0;
    int32_t left_rpm = left_speed / wheel_circumference * 60.0;
    // std::cout<<"------ closed loop: right rpm: " << right_rpm << " left rpm: " << left_rpm << std::endl;
    ROS_INFO_STREAM("cmdvel rpm right: " << right_rpm << " left: " << left_rpm);
    // right_cmd << "!S 1 " << right_rpm << "\r";
    // left_cmd << "!S 2 " << left_rpm << "\r";
    right_cmd = right_rpm;
    left_cmd = left_rpm;
  }

// split int32_t to uint16
// publish motor rpm to modbus
plc_modbus_node::UInt16Array speed_write;
speed_write.name = "roboteq";
speed_write.data.push_back((left_cmd & 0xFFFF0000)>>16);
speed_write.data.push_back((left_cmd & 0x0000FFFF));
speed_write.data.push_back((right_cmd & 0xFFFF0000)>>16);
speed_write.data.push_back((right_cmd & 0x0000FFFF));

regs_write.publish(speed_write);
}

void MainNode::regs_write_setup()
{
  ROS_INFO_STREAM("Publishing to topic " << regs_write_topic);
  regs_write = nh.advertise<plc_modbus_node::UInt16Array>(regs_write_topic, 100);
}

void MainNode::sensor_setup()
{
  ROS_INFO_STREAM("Subscribing to topic " << sensors_topic);
  sensor_sub = nh.subscribe<plc_modbus_node::roboteq_sensors>(sensors_topic, 100, &MainNode::sensor_callback, this);
}

void MainNode::sensor_callback(const roboteq_sensors::ConstPtr& data)
{
  rb_sensors = *data;
}

void MainNode::cmdvel_setup()
{
  ROS_INFO_STREAM("Subscribing to topic " << cmdvel_topic);
  cmdvel_sub = nh.subscribe(cmdvel_topic, 1000, &MainNode::cmdvel_callback, this);
}

// void MainNode::cmdvel_loop()
// {
// }

// void MainNode::cmdvel_run()
// {
// }


//
// odom publisher
//

void MainNode::odom_setup()
{

  if ( pub_odom_tf )
  {
    ROS_INFO("Broadcasting odom tf");
//    odom_broadcaster.init(nh);	// ???
  }

  ROS_INFO_STREAM("Publishing to topic " << odom_topic);
  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);

  tf_msg.header.seq = 0;
  tf_msg.header.frame_id = odom_frame;
  tf_msg.child_frame_id = base_frame;

  odom_msg.header.seq = 0;
  odom_msg.header.frame_id = odom_frame;
  odom_msg.child_frame_id = base_frame;

  odom_msg.pose.covariance.assign(0);
  odom_msg.pose.covariance[0] = 0.001;
  odom_msg.pose.covariance[7] = 0.001;
  odom_msg.pose.covariance[14] = 1000000;
  odom_msg.pose.covariance[21] = 1000000;
  odom_msg.pose.covariance[28] = 1000000;
  odom_msg.pose.covariance[35] = 1000;

  odom_msg.twist.covariance.assign(0);
  odom_msg.twist.covariance[0] = 0.001;
  odom_msg.twist.covariance[7] = 0.001;
  odom_msg.twist.covariance[14] = 1000000;
  odom_msg.twist.covariance[21] = 1000000;
  odom_msg.twist.covariance[28] = 1000000;
  odom_msg.twist.covariance[35] = 1000;


  odom_last_time = millis();
  current_last_time = millis();
}

// void MainNode::odom_stream()
// {
// }

void MainNode::odom_loop()
{
  uint32_t nowtime = millis();

  // get encoder counts from roboteq_sensors
  odom_encoder_right = rb_sensors.encoder_right;
  odom_encoder_left = rb_sensors.encoder_left;

  odom_publish();
}

//void MainNode::odom_hs_run()
//{
//}

// void MainNode::odom_ms_run()
// {
// }

// void MainNode::odom_ls_run()
// {
// }

void MainNode::odom_publish()
{
  // return;
  // determine delta time in seconds
  uint32_t nowtime = millis();
  //float dt = (float)DELTAT(nowtime,odom_last_time) / 1000.0;
  float dt = (float)rb_sensors.time_elapsed;
  odom_last_time = nowtime;

#ifdef _ODOM_DEBUG
/*
ROS_DEBUG("right: ");
ROS_DEBUG(odom_encoder_right);
ROS_DEBUG(" left: ");
ROS_DEBUG(odom_encoder_left);
ROS_DEBUG(" dt: ");
ROS_DEBUG(dt);
ROS_DEBUG("");
*/
#endif

#ifdef _ODOM_USE_RPM
  float linear = (((float)odom_rpm_right / (float)60.0f) * wheel_circumference
      + ((float)odom_rpm_left / 60.0f) * wheel_circumference) * dt / 2.0;
//  float angular = ((float)odom_encoder_right / (float)encoder_cpr * wheel_circumference - (float)odom_encoder_left / (float)encoder_cpr * wheel_circumference) / track_width * -1.0;
  float angular = (((float)odom_rpm_right / (float)60.0f) * wheel_circumference 
      - ((float)odom_rpm_left / 60.0f) * wheel_circumference) *dt / track_width;
#else
  ROS_INFO_STREAM("use encoder count");
  // determine deltas of distance and angle
  float linear = ((float)odom_encoder_right / (float)encoder_cpr * wheel_circumference + (float)odom_encoder_left / (float)encoder_cpr * wheel_circumference) / 2.0;
  // float angular = ((float)odom_encoder_right / (float)encoder_cpr * wheel_circumference - (float)odom_encoder_left / (float)encoder_cpr * wheel_circumference) / track_width * -1.0;
  float angular = ((float)odom_encoder_right / (float)encoder_cpr * wheel_circumference - (float)odom_encoder_left / (float)encoder_cpr * wheel_circumference) / track_width;
#endif

  // Update odometry
  odom_x += linear * cos(odom_yaw);        // m
  odom_y += linear * sin(odom_yaw);        // m
  odom_yaw = NORMALIZE(odom_yaw + angular);  // rad
#ifdef _ODOM_DEBUG
  ROS_INFO_STREAM( "linear: " << linear << " angular: " << angular);
  ROS_INFO_STREAM( "odom x: " << odom_x << " y: " << odom_y << " yaw: " << odom_yaw);
#endif

  // Calculate velocities
  float vx = (odom_x - odom_last_x) / dt;
  float vy = (odom_y - odom_last_y) / dt;
  float vyaw = (odom_yaw - odom_last_yaw) / dt;

  odom_last_x = odom_x;
  odom_last_y = odom_y;
  odom_last_yaw = odom_yaw;

  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odom_yaw);

  if ( pub_odom_tf )
  {
    tf_msg.header.seq++;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.transform.translation.x = odom_x;
    tf_msg.transform.translation.y = odom_y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = quat;
    odom_broadcaster.sendTransform(tf_msg);
  }

  odom_msg.header.seq++;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = quat;
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = vyaw;
  odom_pub.publish(odom_msg);
}


int MainNode::run()
{

	ROS_INFO("Beginning setup...");
  

	cmdvel_setup();
	odom_setup();
  regs_write_setup();
  sensor_setup();

  // TODO: Check if roboteq sensors are subscribed with data

  starttime = millis();
  // hstimer = starttime;
  // mstimer = starttime;
  // lstimer = starttime;

//  ros::Rate loop_rate(10);

  ROS_INFO("Beginning looping...");
	
  while (ros::ok())
  {

    // cmdvel_loop();
    odom_loop();

    uint32_t nowtime = millis();
//ROS_INFO_STREAM("loop nowtime: " << nowtime << " lstimer: " << lstimer << " delta: " << DELTAT(nowtime,lstimer) << " / " << (nowtime-lstimer));
//uint32_t delta = DELTAT(nowtime,lstimer);
//ROS_INFO_STREAM("loop nowtime: " << nowtime << " lstimer: " << lstimer << " delta: " << delta << " / " << (nowtime-lstimer));

//    // Handle 50 Hz publishing
//    if (DELTAT(nowtime,hstimer) >= 20)
    // Handle 30 Hz publishing
//     if (DELTAT(nowtime,hstimer) >= 33)
//     {
//       hstimer = nowtime;
//       odom_hs_run();
//     }

    // Handle 10 Hz publishing
    // if (DELTAT(nowtime,mstimer) >= 100)
    // {
    //   mstimer = nowtime;
    //   cmdvel_run();
    //   odom_ms_run();
    // }

    // Handle 1 Hz publishing
    // if (DELTAT(nowtime,lstimer) >= 1000)
    // {
    //   lstimer = nowtime;
    //   odom_ls_run();
    // }

    ros::spinOnce();

//    loop_rate.sleep();
  }
	

  ROS_INFO("Exiting");
	
  return 0;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "roboteq_modbus_node");

  MainNode node;

  return node.run();
}

