#include "ros/ros.h"
#include "swc_msgs/Control.h"
#include "swc_msgs/Waypoints.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"
#include "math.h"

#define PI 3.14159265
#define DEG2RAD PI/180.0
ros::Publisher g_control_pub;

double my_heading = 0;

swc_msgs::Gps cur_pos;

swc_msgs::Gps final_pos;

tf::Vector3 lidar_field_out;

bool receivedImu = false;
bool receivedGPS = false;
bool receivedLIDAR = false;

double getAngleDiff(double x, double y) {
    return atan2(sin(x-y), cos(x-y));
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    receivedImu = true;

    double roll, pitch, yaw;

    tf::Quaternion qt;
    
    tf::quaternionMsgToTF(imu_msg->orientation, qt);
    tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);

    my_heading = yaw;
}

void gpsCallback(const swc_msgs::Gps::ConstPtr& gps_msg) {
    receivedGPS = true;

    cur_pos = *gps_msg;
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar_msg) {
    receivedLIDAR = true;

    tf::Vector3 lidar_field;
    for (int i=0; i<360; i++) {
        if (lidar_msg->ranges[i] < 0.1) {
            continue;
        }
        if (i > 10 && i < 350) {
            lidar_field.setX(lidar_field.getX() - cos(i * DEG2RAD) / (lidar_msg->ranges[i] * lidar_msg->ranges[i]));
            lidar_field.setY(lidar_field.getY() - sin(i * DEG2RAD) / (lidar_msg->ranges[i] * lidar_msg->ranges[i]));
        } else {
            lidar_field.setX(lidar_field.getX() - cos(i * DEG2RAD) / (lidar_msg->ranges[i] * lidar_msg->ranges[i]));
            lidar_field.setY(lidar_field.getY() - 2 * cos(i * DEG2RAD) / (lidar_msg->ranges[i] * lidar_msg->ranges[i])); // LOL
        }
    }
    lidar_field_out = lidar_field;
}

void controlTimerCallback(const ros::TimerEvent& timer_event) {
    if (!receivedLIDAR || !receivedGPS || !receivedImu) {
        return;
    }

    // Create a new message with speed 1 (m/s) and turn angle 15 (degrees CW)
    swc_msgs::Control controlMsg;

    tf::Vector3 waypoint_field(final_pos.latitude - cur_pos.latitude,  cur_pos.longitude - final_pos.longitude, 0);
    waypoint_field = waypoint_field.rotate(tf::Vector3(0,0,1), my_heading + PI/2.0);


    tf::Vector3 final_field;
    final_field = (waypoint_field.normalized() + 3.0 * lidar_field_out.normalized()).normalized();

    controlMsg.speed = 4.0f + final_field.getX();
    controlMsg.turn_angle = -3.0 * atan2(final_field.getX(), final_field.getY());

    std::cout << "speed: " << controlMsg.speed << std::endl;
    std::cout << "heading: " << my_heading << std::endl;
    // std::cout << "turn angle: " << controlMsg.turn_angle << std::endl;

    // Publish the message to /sim/control so the simulator receives it
    g_control_pub.publish(controlMsg);
}

int main(int argc, char **argv)
{
    // Initalize our node in ROS
    ros::init(argc, argv, "nav_node");
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

    // Subscribe to Laser Scan
    ros::Subscriber lidar_subscriber = node.subscribe<sensor_msgs::LaserScan>("/scan", 1, lidarCallback);

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