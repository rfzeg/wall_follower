/*
Follow Wall Algorithm
Author: Roberto Zegers
Date: 2019-March-27
*/

// ROS Libraries
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h" // Laser Data

// ROS Subscriber:Laser Data, and Messages:Laser Messages
ros::Subscriber laser_subscriber;
sensor_msgs::LaserScan laser_msg;

// The laser_callback function is called each time laser scan data is received
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    // Read and process laser scan values
    laser_msg = *scan_msg;
    // declare an array of float values to keep track of the laser measurements
    std::vector<float> laser_ranges;
    // important note: array of laser range values is filled from right to left (CCW)
    // according to rostopic list 45*16 = 720 individual laser rays are published 
    laser_ranges = laser_msg.ranges;
    // the total number of laser rays the laser range finder has
    size_t range_size = laser_ranges.size();
    // ROS_INFO_ONCE("Number of laser rays: [%d]", range_size); // for debugging
    // variables to store closest (min) distance values on each zone 
    float right_range_min= laser_msg.range_max;
    float fright_range_min= laser_msg.range_max;
    float front_range_min= laser_msg.range_max;
    float fleft_range_min= laser_msg.range_max;
    float left_range_min= laser_msg.range_max;

    float range_max = laser_msg.range_min;
    // cycle trough all laser range rays
    for (size_t i = 0; i < range_size; i++) {
 
        // rays < 144, laser rays to the far right side
        if (i < range_size / 5) {
           // get the smallest (closest) laser range value 
           if (laser_ranges[i] < right_range_min) {
            right_range_min = laser_ranges[i];
            }
        }
        // rays >= 144 and rays < 288, laser rays to the front-right side
        else if (i >= range_size / 5 && i < range_size * 2 / 5) {
           // get the smallest (closest) laser range value 
           if (laser_ranges[i] < fright_range_min) {
            fright_range_min = laser_ranges[i];
            }
        }
        // rays >= 288 and rays < 432, laser rays to the front
        else if (i >= range_size * 2 / 5 && i < range_size * 3 / 5) {
           // get the smallest (closest) laser range value 
           if (laser_ranges[i] < front_range_min) {
            front_range_min = laser_ranges[i];;
            }
        }
        // rays >= 432 and rays < 576, laser rays to the front-left side
        else if (i >= range_size * 3 / 5 && i < range_size * 4 / 5) {
           // get the smallest (closest) laser range value 
           if (laser_ranges[i] < fleft_range_min) {
            fleft_range_min = laser_ranges[i];
            }
        }
        // rays > 576 and rays <= 720, laser rays to the far left side
        else if (i >= range_size * 4 / 5 && i <= range_size) {
           // get the smallest (closest) laser range value 
           if (laser_ranges[i] < left_range_min) {
            left_range_min = laser_ranges[i];
            }
        }
        else
        {
        ROS_ERROR("Ray index not found in range size");
        }
    }
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "follow_wall_commands"); // changed node name if required

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Subscribe to the /scan topic and call the laser_callback function
    laser_subscriber = n.subscribe("/scan", 1000, laser_callback); // node subscribes to "scan" topic

    // Infinite loop where the laser_callback function will be called when new laser messages arrives
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
