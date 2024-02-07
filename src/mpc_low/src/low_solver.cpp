#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <string>

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <std_msgs/Int32.h>
#include <gazebo_msgs/SetModelConfiguration.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include <rosgraph_msgs/Clock.h>

#include "acados_solver_holder.cpp"
//#include "acados_solver_holder.h"
// acados

using namespace std;

ofstream myfile;
ofstream myfile_low;
ofstream myperffile;

double gazebo_time;

void gazebo_time_msg(const rosgraph_msgs::Clock::ConstPtr& msg){
    gazebo_time = msg->clock.toSec();
    return;
}

// Introduce class to make safer goal change
class GoalFollower 
{ 
    // Access specifier 
    public: 
    double goal_pose[6]={0.0};
    double current_pose[6]={0.0};
    double tracking_goal[30]={0.0};
    int goal_status = 0;
    double costmap_data[520] = {-100.0};

    // Check pose visualy
    double goal_from_target[7]={0.0};

    void goal_calback(const nav_msgs::Path::ConstPtr& msg){
        int size_of_trajectory = (int)msg->poses.size();
        double angle_goal = 0.0;
        // Now we go throught the trajectory unil we reach 1m or end of trajectory points.
        double trajectory_length = 0.0;
        int goal_index = 0;
        if (size_of_trajectory>1) {
            for (int i=1; i<size_of_trajectory; i++) {
                double x_diff = msg->poses[i].pose.position.x-msg->poses[i-1].pose.position.x;
                double y_diff = msg->poses[i].pose.position.y-msg->poses[i-1].pose.position.y;
                trajectory_length = trajectory_length + sqrt(x_diff*x_diff+y_diff*y_diff);
                goal_index = i;
                if (trajectory_length>=1.0) {
                    break;
                }
            }
        }
        if (size_of_trajectory>1) {
            goal_status = 1;
            tracking_goal[0] = msg->poses[goal_index].pose.position.x;
            tracking_goal[1] = msg->poses[goal_index].pose.position.y;
            angle_goal = 2* std::atan2( msg->poses[goal_index].pose.orientation.z,msg->poses[goal_index].pose.orientation.w);
            if (goal_index<size_of_trajectory-1) angle_goal = std::atan2(msg->poses[goal_index].pose.position.y - msg->poses[goal_index-1].pose.position.y, msg->poses[goal_index].pose.position.x - msg->poses[goal_index-1].pose.position.x);
            tracking_goal[2] = angle_goal;
            // check goal
            goal_from_target[0] = msg->poses[goal_index].pose.position.x;
            goal_from_target[1] = msg->poses[goal_index].pose.position.y;
            goal_from_target[2] = msg->poses[goal_index].pose.position.z;
            goal_from_target[3] = msg->poses[goal_index].pose.orientation.x;
            goal_from_target[4] = msg->poses[goal_index].pose.orientation.y;
            goal_from_target[5] = msg->poses[goal_index].pose.orientation.z;
            goal_from_target[6] = msg->poses[goal_index].pose.orientation.w;
            //
            for (int i=1;i<10;i++) {
                tracking_goal[i*3+0] = tracking_goal[0];
                tracking_goal[i*3+1] = tracking_goal[1];
                tracking_goal[i*3+2] = tracking_goal[2];
            }
            goal_pose[0] = tracking_goal[0];
            goal_pose[1] = tracking_goal[1];
            goal_pose[2] = tracking_goal[2];
        }
        else {
            goal_pose[0] = 0.0;
            goal_pose[1] = 0.0;
            goal_pose[2] = 0.0;
            goal_status = 0;
        }

        ROS_INFO("Trajectory recieved: %i, length %f ", msg->poses.size(), trajectory_length);
        return;
    }

    void rtabmap_pose_calback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        current_pose[0] = msg->pose.position.x;
        current_pose[1] = msg->pose.position.y;
        double angle_current = 2* std::atan2( msg->pose.orientation.z,msg->pose.orientation.w);
        current_pose[2] = angle_current;
        ROS_INFO("Pose recieved ");
        return;
    }

    void amcl_pose_calback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
        current_pose[0] = msg->pose.pose.position.x;
        current_pose[1] = msg->pose.pose.position.y;
        double angle_current = 2* std::atan2( msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
        current_pose[2] = angle_current;
        //ROS_INFO("Trajectory recieved: %i ", msg->poses.size());
        return;
    }
    
   void odom_pose_calback(const nav_msgs::Odometry::ConstPtr& msg){
        current_pose[0] = msg->pose.pose.position.x;
        current_pose[1] = msg->pose.pose.position.y;
        double angle_current = 2* std::atan2( msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
        current_pose[2] = angle_current;
        //ROS_INFO("Trajectory recieved: %i ", msg->poses.size());
        return;
    }


    void costmap_calback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        //ROS_INFO("Map recieved: %i ", msg->data.size());
        int obst_count = 0.0;
        int a = 0;
        int length = msg->data.size();
        // Map 200x200 with resolution of 0.05m
        // we need to convert map index to meters offset, map is 10m by 10m, middle is 5m
        double resolution = msg->info.resolution;
        int matrix_size = msg->info.width;
        for (int i=0;i<length;i++) {
	    if (msg->data[i]>99.0) {
            obst_count=obst_count+1;
            //costmap_data[2*obst_count+0] = ((int)i / 200)*0.05-5.0;
            //costmap_data[2*obst_count+1] = (i % 200)*0.05-5.0;
            costmap_data[2*obst_count+0] = (i % 200)*resolution+msg->info.origin.position.x;
            costmap_data[2*obst_count+1] = ((int)i / 200)*resolution+msg->info.origin.position.y;
            if (obst_count==260) break;
        }
	    //printf("%f",obst_count);
	    a = a+1;
        }
        //printf("%i",obst_count);
        ROS_INFO("Obst info: %i", obst_count);
        ROS_INFO("Map info: %f %i %i %i", msg->info.resolution, msg->info.width, msg->info.height, length);
        return;
    }

}; 


int main(int argc, char **argv)
{
  myfile.open("../../../data_low.csv", ios::out);
  ros::init(argc, argv, "joint_controller_low");

  ros::NodeHandle n;


  ROS_INFO("Node Started");
  //--------------------------------
  GoalFollower my_follower;
  ros::Subscriber goal_topic = n.subscribe("/move_base/TrajectoryPlannerROS/global_plan", 1, &GoalFollower::goal_calback, &my_follower);
  ros::Subscriber costmap_topic = n.subscribe("/move_base/local_costmap/costmap", 1, &GoalFollower::costmap_calback, &my_follower);
  ros::Subscriber rtabmap_pose = n.subscribe("/rtabmap_pose_extruction", 1, &GoalFollower::rtabmap_pose_calback, &my_follower);
  ros::Subscriber amcl_pose_topic = n.subscribe("/amcl_pose", 1, &GoalFollower::amcl_pose_calback, &my_follower);
  ros::Subscriber odom_pose_topic = n.subscribe("/odom",1, &GoalFollower::odom_pose_calback, &my_follower);
  ros::Subscriber gazebo_time_listener = n.subscribe("/clock", 1, gazebo_time_msg);

  ros::Publisher low_level_trajectory = n.advertise<nav_msgs::Path>("mpc_solver/low_level_trajectory", 1);

  ros::Publisher motor_controller = n.advertise<std_msgs::Float64MultiArray>("/joint_group_velocity_controller/command", 1);

  ros::Publisher goal_arrow_pub = n.advertise<geometry_msgs::PoseStamped>("goal_arrow", 1);
  ros::Publisher robot_arrow_pub = n.advertise<geometry_msgs::PoseStamped>("robot_arrow", 1);



  my_NMPC_solver myMpcSolver=my_NMPC_solver(5);

  double cgoal[3];

  double smallest_dist;
  double local_val;
  double min_dist[] = {10000, 10000, 10000, 10000, 10000, 10000, 10000};
  ros::Rate loop_rate(40);

  // Big loop
  while (ros::ok())
  {
    double current_error = (my_follower.goal_pose[0]-my_follower.current_pose[0])*(my_follower.goal_pose[0]-my_follower.current_pose[0])+
(my_follower.goal_pose[1]-my_follower.current_pose[1])*(my_follower.goal_pose[1]-my_follower.current_pose[1])+
(my_follower.goal_pose[2]-my_follower.current_pose[2])*(my_follower.goal_pose[2]-my_follower.current_pose[2]);
    if (my_follower.goal_status || current_error>0.10) {
        // Goal reference position
        double current_robot_position[3];
        double costmap_data[520];
        double current_robot_goal[3];
        for (int i = 0; i < 2; ++i) current_robot_position[ i ] = 0.0;
        current_robot_position[ 2 ] = my_follower.current_pose[2]; // set orientation of the real world
 
        // substiture position from the goal and keep the goal orientation
	current_robot_goal[ 0 ] = my_follower.goal_pose[0]-my_follower.current_pose[0];
	current_robot_goal[ 1 ] = my_follower.goal_pose[1]-my_follower.current_pose[1];
	current_robot_goal[ 2 ] = my_follower.goal_pose[2];
    
        // load constraints for costmap
        for (int i = 0; i < 520; ++i) costmap_data[ i ] = my_follower.costmap_data[i];//-1000.0;
        for (int i = 0; i < 260; i++) {
            costmap_data[ 2*i + 0 ] = costmap_data[ 2*i + 0 ]-my_follower.current_pose[0];
            costmap_data[ 2*i + 1 ] = costmap_data[ 2*i + 1 ]-my_follower.current_pose[1];
        }

        double tracking_goal[30]={0.0};

        for (int i=0;i<10;i++) {
            tracking_goal[i*3+0] = my_follower.goal_pose[0]-my_follower.current_pose[0];
            tracking_goal[i*3+1] = my_follower.goal_pose[1]-my_follower.current_pose[1];
            tracking_goal[i*3+2] = my_follower.goal_pose[2];
        }

        // Check pose visualy
        geometry_msgs::PoseStamped goal_pose;
        
        goal_pose.header.stamp = ros::Time::now();
        goal_pose.header.frame_id = "map";
        goal_pose.pose.position.x = my_follower.goal_pose[0];
        goal_pose.pose.position.y = my_follower.goal_pose[1];
        goal_pose.pose.position.z = 0.0;

        tf2::Quaternion myQuaternion;

        myQuaternion.setRPY(0.0,0.0,my_follower.goal_pose[2]);

        myQuaternion=myQuaternion.normalize();
        goal_pose.pose.orientation.x = myQuaternion.x();
        goal_pose.pose.orientation.y = myQuaternion.y();
        goal_pose.pose.orientation.z = myQuaternion.z();
        goal_pose.pose.orientation.w = myQuaternion.w();
        
        goal_arrow_pub.publish(goal_pose);


        // Check pose visualy
        geometry_msgs::PoseStamped robot_pose;
        
        robot_pose.header.stamp = ros::Time::now();
        robot_pose.header.frame_id = "map";
        robot_pose.pose.position.x = my_follower.goal_from_target[0];
        robot_pose.pose.position.y = my_follower.goal_from_target[1];
        robot_pose.pose.position.z = my_follower.goal_from_target[2];
        robot_pose.pose.orientation.x = my_follower.goal_from_target[3];
        robot_pose.pose.orientation.y = my_follower.goal_from_target[4];
        robot_pose.pose.orientation.z = my_follower.goal_from_target[5];
        robot_pose.pose.orientation.w = my_follower.goal_from_target[6];
        robot_arrow_pub.publish(robot_pose);

        /*ROS_INFO("Goal %f %f %f",my_follower.goal_pose[0],
                  my_follower.goal_pose[1],my_follower.goal_pose[2]); 
        ROS_INFO("State %f %f %f",my_follower.current_pose[0],
                  my_follower.current_pose[1],my_follower.current_pose[2]);
        */
        double result[8]={0.0};
        double trajectory[33]={0.0};
        int status=myMpcSolver.solve_my_mpc(current_robot_position, costmap_data, current_robot_goal, tracking_goal, result, trajectory);
        
        if (status > 0) {
            //ROS_INFO("Destroying solver object");
            myMpcSolver.reset_solver();
            myMpcSolver=my_NMPC_solver(5);
            ROS_INFO("Solver recreated");
        }

        if (status==4) {
              for (int i=0; i<4; i++) result[i] = 0.0;
        }

        double cgoal_trejectory[33];
        for (int i=0;i<11;i++) {
            cgoal_trejectory[3*i+0] = trajectory[3*i+0] + my_follower.current_pose[0];
            cgoal_trejectory[3*i+1] = trajectory[3*i+1] + my_follower.current_pose[1];
            cgoal_trejectory[3*i+2] = 0.0;
            //ROS_INFO("%f %f %f",cgoal_trejectory[3*i+0], cgoal_trejectory[3*i+1], cgoal_trejectory[3*i+2]);
        }
        nav_msgs::Path path;
        path.header.frame_id="map";
        geometry_msgs::PoseStamped pose;
        
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        for (int i=0;i<11;i++) {
            pose.pose.position.x = cgoal_trejectory[3*i+0];
            pose.pose.position.y = cgoal_trejectory[3*i+1];
            pose.pose.position.z = cgoal_trejectory[3*i+2];
            
            path.poses.push_back(pose);
        }
         
        low_level_trajectory.publish(path);
        std_msgs::Float64MultiArray wheel_command;
        wheel_command.data.push_back(result[0]);
        wheel_command.data.push_back(result[0]);
        wheel_command.data.push_back(result[1]);
        wheel_command.data.push_back(result[1]);
        motor_controller.publish(wheel_command);
        ROS_INFO("Time %f; Status %i; Error: %f",result[5], status, current_error);
        ROS_INFO("Vels %f %f", result[0], result[1]);
        ROS_INFO("Goal %f %f %f", current_robot_goal[0], current_robot_goal[1], current_robot_goal[2]);
        ROS_INFO("Pose %f %f %f", my_follower.current_pose[0], my_follower.current_pose[1], my_follower.current_pose[2]);
        
    }
    //ROS_INFO("goal_status %i; current_error %f", my_follower.goal_status, current_error);
    loop_rate.sleep();
    ros::spinOnce();
  }
  //** end of Loop

  myfile.close();
  return 0;
}

