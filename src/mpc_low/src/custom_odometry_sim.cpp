/*
 * Automatic Addison
 * Date: May 20, 2021
 * ROS Version: ROS 1 - Melodic
 * Website: https://automaticaddison.com
 * Publishes odometry information for use with robot_pose_ekf package.
 *   This odometry information is based on wheel encoder tick counts.
 * Subscribe: ROS node that subscribes to the following topics:
 *  right_ticks : Tick counts from the right motor encoder (std_msgs/Int16)
 * 
 *  left_ticks : Tick counts from the left motor encoder  (std_msgs/Int16)
 * 
 *  initial_2d : The initial position and orientation of the robot.
 *               (geometry_msgs/PoseStamped)
 *
 * Publish: This node will publish to the following topics:
 *  odom_data_euler : Position and velocity estimate. The orientation.z 
 *                    variable is an Euler angle representing the yaw angle.
 *                    (nav_msgs/Odometry)
 *  odom_data_quat : Position and velocity estimate. The orientation is 
 *                   in quaternion format.
 *                   (nav_msgs/Odometry)
 * Modified from Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */
 
// Include various libraries
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <sensor_msgs/JointState.h>
 
// Create odometry data publishers
ros::Publisher odom_data_pub;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;
 
// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;
 
// Robot physical constants
const double TICKS_PER_REVOLUTION = 620; // For reference purposes.
const double WHEEL_RADIUS = 0.033; // Wheel radius in meters
const double WHEEL_BASE = 0.17; // Center of left tire to center of right tire
const double TICKS_PER_METER = 3100; // Original was 2800
 
// Distance both wheels have traveled
double distanceLeft = 0;
double distanceRight = 0;
 
// Flag to see if initial pose has been received
bool initialPoseRecieved = false;
 
using namespace std;
 
// Get initial_2d message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {
 
  odomOld.pose.pose.position.x = rvizClick.pose.position.x;
  odomOld.pose.pose.position.y = rvizClick.pose.position.y;
  odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
  initialPoseRecieved = true;
}
 
// Update odometry information
void update_odom(const sensor_msgs::JointState &msg) {
  // Specific constants
  double B = 0.3765; // distance between the wheels
  double alpha = 1.8; // handpicked parameter
  double r_wheel = 0.10; // wheel radius
  double scale = 1.0/(alpha*B);
  // Here we convert incoming message to right and left velocity
  double message[2]={0.0, 0.0};
  message[0]=(0.5*msg.velocity[1]+0.5*msg.velocity[3]); // avg of right wheels
  message[1]=(0.5*msg.velocity[0]+0.5*msg.velocity[2]); // avg of left wheels
  odomNew.header.stamp = ros::Time::now();
  double delta_time = (odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
  // Calculate the average distance
  double cycleDistance = (0.5*r_wheel*message[0])+(0.5*r_wheel*message[1])*delta_time;
   
  // Calculate the number of radians the robot has turned since the last cycle
  double cycleAngle = (1.0*scale*message[0])+(-1.0*scale*message[1])*delta_time;
 
  // Average angle during the last cycle
  double avgAngle = cycleAngle/2 + odomOld.pose.pose.orientation.z;
     
  if (avgAngle > PI) {
    avgAngle -= 2*PI;
  }
  else if (avgAngle < -PI) {
    avgAngle += 2*PI;
  }
  else{}
 
  // Calculate the new pose (x, y, and theta)
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
  odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;
 
  // Prevent lockup from a single bad cycle
  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
     || isnan(odomNew.pose.pose.position.z)) {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }
 
  // Make sure theta stays in the correct range
  if (odomNew.pose.pose.orientation.z > PI) {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  }
  else if (odomNew.pose.pose.orientation.z < -PI) {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }
  else{}
 
  // Compute the velocity

  // here we replaced the dynamics
  odomNew.twist.twist.linear.x = (0.5*r_wheel*message[0])+(0.5*r_wheel*message[1]);
  odomNew.twist.twist.angular.z = (1.0*scale*message[0])+(-1.0*scale*message[1]);
 
  // Save the pose data for the next cycle
  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;
 
  // Publish the odometry message
  odom_data_pub.publish(odomNew);
}
 
int main(int argc, char **argv) {
   
  // Set the data fields of the odometry message
  odomNew.header.frame_id = "odom";
  odomNew.pose.pose.position.z = 0;
  odomNew.pose.pose.orientation.x = 0;
  odomNew.pose.pose.orientation.y = 0;
  odomNew.twist.twist.linear.x = 0;
  odomNew.twist.twist.linear.y = 0;
  odomNew.twist.twist.linear.z = 0;
  odomNew.twist.twist.angular.x = 0;
  odomNew.twist.twist.angular.y = 0;
  odomNew.twist.twist.angular.z = 0;
  odomOld.pose.pose.position.x = initialX;
  odomOld.pose.pose.position.y = initialY;
  odomOld.pose.pose.orientation.z = initialTheta;
 
  // Launch ROS and create a node
  ros::init(argc, argv, "ekf_odom_pub");
  ros::NodeHandle node;
 
  // Subscribe to ROS topics
  ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, set_initial_2d);
 
  // Publisher of simple odom message where orientation.z is an euler angle
  odom_data_pub = node.advertise<nav_msgs::Odometry>("/jackal_velocity_controller/odom", 100);

  // Subscribe to ROS topics
  ros::Subscriber sim_updater = node.subscribe("/joint_states", 1, update_odom);

 
  ros::Rate loop_rate(100); 
     
  initialPoseRecieved = true;
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
 
  return 0;
}
