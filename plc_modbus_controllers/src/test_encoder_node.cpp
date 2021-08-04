#include <ros/ros.h>
#include <std_msgs/UInt32.h>

uint32_t prev_odom_encoder_right = 0;
uint32_t odom_encoder_right = 0;

int32_t rel_odom_encoder_right;

void sub_callback(const std_msgs::UInt32::ConstPtr& data) {
      // get encoder counts from roboteq_sensors
  odom_encoder_right = data->data;
ROS_INFO("prev encoder val: %u", prev_odom_encoder_right);
ROS_INFO("new encoder val: %u", odom_encoder_right);

  // Calculating relative encoder counts
  int buffer = 200000;
  if ((prev_odom_encoder_right >= (UINT_MAX - buffer)) && (odom_encoder_right <= buffer))
  {
    // crossing upper limit
    rel_odom_encoder_right = (UINT_MAX - prev_odom_encoder_right) + odom_encoder_right + 1;
  }
  else if ((prev_odom_encoder_right <= buffer) && (odom_encoder_right >= (UINT_MAX - buffer)))
  {
    // crossing lower limit
    rel_odom_encoder_right = -(prev_odom_encoder_right + (UINT_MAX - odom_encoder_right) + 1);
  }
  else{
    rel_odom_encoder_right = odom_encoder_right - prev_odom_encoder_right;
  }

ROS_INFO("relative encoder val: %d", rel_odom_encoder_right);
  prev_odom_encoder_right = odom_encoder_right;

}

int main(int argc, char **argv) {
      ros::init(argc, argv, "test_node");
ros::NodeHandle nh("~");
ros::Subscriber sub = nh.subscribe("/test", 100, sub_callback);

ros::spin();

return 0;
}