/*
Basic Follow Wall Algorithm
Author: Roberto Zegers
Date: 2019-March-27
*/

// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> // Laser Data
#include <geometry_msgs/Twist.h> // Motor Commands

// ROS Subscriber for laser data and messages type (LaserScan)
ros::Subscriber laser_subscriber;
sensor_msgs::LaserScan laser_msg;

// ROS Publisher for motor commands and message type (Twist)
ros::Publisher motor_command_publisher;
geometry_msgs::Twist motor_command;

// global array to keep track of the min. distance value on each zone
float zone[5];
// global variable used to set the robots's driving logic
int state;

// The laser_callback function is called each time laser scan data is received
void laser_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  // Read and process laser scan values
  laser_msg = *scan_msg;
  // declare an array of float values to keep track of the laser measurements
  std::vector<float> laser_rays;
  // note: array of laser range values is filled from right to left (CCW)
  // rostopic list shows 45*16 = 720 individual laser rays published
  laser_rays = laser_msg.ranges;
  // the total number of laser rays the laser range finder has
  size_t range_size = laser_rays.size();
  ROS_INFO_ONCE("Number of laser rays: [%zu]", range_size); // for debugging
  // variables to store closest (min) distance values on each zone
  zone[0] = laser_msg.range_max;
  zone[1] = laser_msg.range_max;
  zone[2] = laser_msg.range_max;
  zone[3] = laser_msg.range_max;
  zone[4] = laser_msg.range_max;

  float range_max = laser_msg.range_min;
  // cycle trough all laser range rays
  for (size_t i = 0; i < range_size; i++) {

    // rays < 144, laser rays to the far right side
    if (i < range_size / 5) {
      // get the smallest (closest) laser range value
      if (laser_rays[i] < zone[0]) {
        zone[0] = laser_rays[i];
      }
    }
    // rays >= 144 and rays < 288, laser rays to the front-right side
    else if (i >= range_size / 5 && i < range_size * 2 / 5) {
      // get the smallest (closest) laser range value
      if (laser_rays[i] < zone[1]) {
        zone[1] = laser_rays[i];
      }
    }
    // rays >= 288 and rays < 432, laser rays to the front
    else if (i >= range_size * 2 / 5 && i < range_size * 3 / 5) {
      // get the smallest (closest) laser range value
      if (laser_rays[i] < zone[2]) {
        zone[2] = laser_rays[i];
      }
    }
    // rays >= 432 and rays < 576, laser rays to the front-left side
    else if (i >= range_size * 3 / 5 && i < range_size * 4 / 5) {
      // get the smallest (closest) laser range value
      if (laser_rays[i] < zone[3]) {
        zone[3] = laser_rays[i];
      }
    }
    // rays > 576 and rays <= 720, laser rays to the far left side
    else if (i >= range_size * 4 / 5 && i <= range_size) {
      // get the smallest (closest) laser range value
      if (laser_rays[i] < zone[4]) {
        zone[4] = laser_rays[i];
      }
    } else {
      ROS_ERROR("Ray index not found in range size");
    }
  } // end of for
  // ROS_INFO("Closest object to the far right: [%f]: ", zone[0]);
  // ROS_INFO("Closest object to the front-right: [%f]: ", zone[1]);
  // ROS_INFO("Closest object to the front: [%f]: ", zone[2]);
  // ROS_INFO("Closest object to the front-left: [%f]: ", zone[3]);
  // ROS_INFO("Closest object to the far left: [%f]: ", zone[4]);
}

/*
Detemine velocity commands for each state and fill in a Twist message
*/

void robot_move(geometry_msgs::Twist &motor_command) {
  // ROS_INFO("Wall follower state: [%d]", state);
  switch (state) {
  case 0:
    // Find a wall: turn CW (right) while moving ahead
    motor_command.linear.x = 0.15;
    motor_command.angular.z = -0.15;
    break;

  case 1:
    // Turn left
    motor_command.linear.x = 0.0;
    motor_command.angular.z = 0.2;
    break;

  case 2:
    // Follow the wall: keep moving straight ahead
    motor_command.angular.z = 0.0;
    motor_command.linear.x = 0.15;
    break;
  }
}

void drive_logic() {
  float linear_x = 0;
  float angular_z = 0;
  int state;
  // fine tune distance (mt) used to consider a region as blocked by an obstacle
  int d = 2.0;

  if (zone[2] > d && zone[3] > d && zone[1] > d) {
    ROS_INFO("case 1 - nothing");
    state = 0;
  } else if (zone[2] < d && zone[3] > d && zone[1] > d) {
    ROS_INFO("case 2 - front");
    state = 1;
  } else if (zone[2] > d && zone[3] > d && zone[1] < d) {
    ROS_INFO("case 3 - front-right");
    state = 2;
  } else if (zone[2] > d && zone[3] < d && zone[1] > d) {
    ROS_INFO("case 4 - front-left");
    state = 0;
  } else if (zone[2] < d && zone[3] > d && zone[1] < d) {
    ROS_INFO("case 5 - front and front-right");
    state = 1;
  } else if (zone[2] < d && zone[3] < d && zone[1] > d) {
    ROS_INFO("case 6 - front and front-left");
    state = 1;
  } else if (zone[2] < d && zone[3] < d && zone[1] < d) {
    ROS_INFO("case 7 - front, front-left and front-right");
    state = 1;
  } else if (zone[2] > d && zone[3] < d && zone[1] < d) {
    ROS_INFO("case 8 - front-left and front-right");
    state = 0;
  } else {
    ROS_INFO("Unknown case");
  }
}

int main(int argc, char **argv) {
  // Initialize a ROS node, change name if required
  ros::init(argc, argv, "follow_wall_commands");

  // Create a ROS NodeHandle object
  ros::NodeHandle n;

  // Inform ROS master that we will be publishing a message of type
  // geometry_msgs::Twist on the robot actuation topic with a publishing queue
  // size of 100
  motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

  // Subscribe to the /scan topic and call the laser_callback function
  laser_subscriber = n.subscribe("/scan", 1000, laser_callback);
  ROS_INFO_ONCE("Hello World! Follow Wall Customized Version!");
  // Loop, the laser_callback function is called when new laser messages arrives
  while (ros::ok()) {
    ros::spinOnce(); // call all the callbacks waiting to be called
    // take action: determine the state of the environment surroundings
    // and set the logic used to drive the robot
    drive_logic();
    // Fill in motor command message and publish it
    robot_move(motor_command);
    // Publish motor commands to the robot and wait 10ms
    motor_command_publisher.publish(motor_command);
    usleep(10);
  }
  return 0;
}
