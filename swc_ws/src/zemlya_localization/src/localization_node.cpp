#include "ros/ros.h"
#include "swc_msgs/Control.h"
#include "swc_msgs/Waypoints.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"
#include "math.h"

#define PI 3.14159265
#define DEG2RAD PI/180.0

bool receivedImu = false;
bool receivedGPS = false;

double getAngleDiff(double x, double y) {
    return atan2(sin(x-y), cos(x-y));
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    receivedImu = true;

    double roll, pitch, yaw;

    tf::Quaternion qt;
    
    tf::quaternionMsgToTF(imu_msg->orientation, qt);
    tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);
}

void gpsCallback(const swc_msgs::Gps::ConstPtr& gps_msg) {
    receivedGPS = true;
}

void timerCallback(const ros::TimerEvent& timer_event) {
    // TODO: localization
}

int main(int argc, char **argv)
{
    // Initalize our node in ROS
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle node;

    // Subscribe to Imu
    ros::Subscriber imu_subscriber = node.subscribe<sensor_msgs::Imu>("/sim/imu", 1, imuCallback);

    // Subscribe to GPS
    ros::Subscriber gps_subscriber = node.subscribe<swc_msgs::Gps>("/sim/gps", 1, gpsCallback);

    // Create a timer that calls timer_callback() with a period of 0.05 (20 Hz)
    ros::Timer control_timer = node.createTimer(ros::Duration(0.05), &timerCallback, false);

    // Let ROS take control of this thread until a ROS wants to kill
    ros::spin();

    return 0;
}