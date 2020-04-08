#include "ros/ros.h"
#include "swc_msgs/Control.h"
#include "swc_msgs/Waypoints.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include "math.h"

#define PI 3.14159265
ros::Publisher g_control_pub;

double my_heading = 0;

swc_msgs::Gps cur_pos;
swc_msgs::Gps final_pos;

double getAngleDiff(double x, double y) {
    return atan2(sin(x-y), cos(x-y));
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    double roll, pitch, yaw;

    tf::Quaternion qt;
    
    tf::quaternionMsgToTF(imu_msg->orientation, qt);
    tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);

    my_heading = yaw;
}

void gpsCallback(const swc_msgs::Gps::ConstPtr& gps_msg) {
    cur_pos = *gps_msg;
}

void controlTimerCallback(const ros::TimerEvent& timer_event) {
    // Create a new message with speed 1 (m/s) and turn angle 15 (degrees CW)
    swc_msgs::Control controlMsg;
    controlMsg.speed = 4;

    double headingToGPS = atan2(final_pos.longitude - cur_pos.longitude, final_pos.latitude - cur_pos.latitude);

    std::cout << "Heading to GPS: " << headingToGPS << std::endl;
    std::cout << "Current Heading: " << my_heading << std::endl;

    controlMsg.turn_angle = -3.0 * getAngleDiff(headingToGPS, my_heading);

    // Publish the message to /sim/control so the simulator receives it
    g_control_pub.publish(controlMsg);
}

int main(int argc, char **argv)
{
    // Initalize our node in ROS
    ros::init(argc, argv, "cpp_robot_control_node");
    ros::NodeHandle node;

    // Create a Publisher that we can use to publish messages to the /sim/control topic
    g_control_pub = node.advertise<swc_msgs::Control>(node.resolveName("/sim/control"), 1);

    // Create and wait for Waypoints service
    ros::ServiceClient waypoint_service = node.serviceClient<swc_msgs::Waypoints>("/sim/waypoints");
    waypoint_service.waitForExistence();

    // Subscribe to Imu
    ros::Subscriber imu_subscriber = node.subscribe<sensor_msgs::Imu>("/sim/imu", 1, imuCallback);

    // Subscribe to GPS
    ros::Subscriber gps_subscriber = node.subscribe<swc_msgs::Gps>("/sim/gps", 1, gpsCallback);

    // Request waypoints and display them
    swc_msgs::Waypoints waypoints_msg;
    waypoint_service.call(waypoints_msg);

    std::cout << "Found the following waypoints:" << std::endl;
    for (int i=0; i<waypoints_msg.response.waypoints.size(); i++) {
        std::cout << waypoints_msg.response.waypoints[i] << std::endl;
    }

    final_pos = waypoints_msg.response.waypoints[4];

    // Create a timer that calls timer_callback() with a period of 0.1 (10 Hz)
    ros::Timer control_timer = node.createTimer(ros::Duration(0.1), &controlTimerCallback, false);

    // Let ROS take control of this thread until a ROS wants to kill
    ros::spin();

    return 0;
}