#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "device_diagnostics/device_connection.h"

device_diagnostics::device_connection connection_statuses;

void diagnostics_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& data) {

}
void d435_back_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& data) {
    connection_statuses.d435_back = true;
}
void d435_front_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& data) {
    connection_statuses.d435_front = true;
}
void hokuyo_frontleft_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& data) {
    connection_statuses.laser_frontleft = true;
}
void hokuyo_backright_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& data) {
    connection_statuses.laser_backright = true;
}
void ouster_scan_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& data) {
    connection_statuses.ouster = true;
}

void reset() {
    connection_pub.d435_back = false;
    connection_pub.d435_front = false;
    connection_pub.laser_frontleft = false;
    connection_pub.laser_backright = false;
    connection_pub.ouster = false;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;

    // Subscribe to diagnostic topics or scan
    ros::Subscriber diagnostics_sub = nh.subscribe("/diagnostics", 100, diagnostics_callback);
    ros::Subscriber d435_back_heartbeat_sub = nh.subscribe("/caato_0/d435_back/realsense2_camera_manager/bond", 100, d435_back_callback);
    ros::Subscriber d435_front_heartbeat_sub = nh.subscribe("/caato_0/d435_front/realsense2_camera_manager/bond", 100, d435_front_callback);
    ros::Subscriber hokuyo_frontleft_sub = nh.subscribe("/caato_0/front_left/scan", 100, hokuyo_frontleft_callback);
    ros::Subscriber hokuyo_backright_sub = nh.subscribe("/caato_0/back_right/scan", 100, hokuyo_backright_callback);
    ros::Subscriber ouster_scan_sub = nh.subscribe("/caato_0/ouster/scan", 100, ouster_scan_callback);

    // Publisher(s) for device statuses
    ros::Publisher connection_pub = nh.advertise<device_diagnostics::device_connection>("/device_diagnostics/connection", 100);

    ros::Rate loop_rate = ros::Rate(1);   // 1 Hz
    while (ros::ok()) {
        // Publish device statuses
        connection_pub.publish(connection_statuses);

        // Reset device statuses
        reset();

        // Loop rate
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
