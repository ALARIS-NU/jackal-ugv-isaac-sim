// ROS
#include <ros/ros.h>

// For visualizing things in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

// C++
#include <string>
#include <vector>


  // For visualizing things in rviz
namespace rvt = rviz_visual_tools;

class Cost_map_tracker
{ 
    // Access specifier 
    public: 
    double costmap_data[8000] = {-100.0};

    void costmap_calback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        //ROS_INFO("Map recieved: %i ", msg->data.size());
        int obst_count = 0.0;
        int length = msg->data.size();
        // Map 200x200 with resolution of 0.05m
        // we need to convert map index to meters offset, map is 10m by 10m, middle is 5m
        for (int i=0;i<length;i++) {
            if (msg->data[i]>99.0) {
                obst_count=obst_count+1;
                costmap_data[2*obst_count+0] = (i % 200)*0.05+msg->info.origin.position.x;
                costmap_data[2*obst_count+1] = ((int)i / 200)*0.05+msg->info.origin.position.y;
                if (obst_count==4000) break;
            }
        }
        ROS_INFO("Obst info: %i", obst_count);
        ROS_INFO("Map info: %f %i %i %i", msg->info.resolution, msg->info.width, msg->info.height, length);
        ROS_INFO("origin %f, %f, %f", msg->info.origin.position.x, msg->info.origin.position.y, msg->info.origin.position.z);
        ROS_INFO("orientation %f, %f, %f, %f", msg->info.origin.orientation.x, msg->info.origin.orientation.y, msg->info.origin.orientation.z, msg->info.origin.orientation.w);
        return;
    }


}; 


int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_tools_demo");
    ROS_INFO_STREAM("Visual Tools Demo");

    // Allow the action server to recieve and send ros messages
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //rviz_visual_tools::RvizVisualToolsDemo demo;

    // A shared node handle
    ros::NodeHandle nh_;

    Cost_map_tracker my_follower;

    ros::Subscriber costmap_topic = nh_.subscribe("/move_base/local_costmap/costmap", 1, &Cost_map_tracker::costmap_calback, &my_follower);

    // For visualizing things in rviz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    std::string name_;

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map","/rviz_visual_markers"));
    //visual_tools_.reset(new rvt::RvizVisualTools("world", "/rviz_visual_tools"));
    visual_tools_->loadMarkerPub();  // create publisher before waiting

    ROS_INFO("Sleeping 5 seconds before running demo");
    ros::Duration(5.0).sleep();

    // Clear messages
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();
    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();



    /*
    ROS_INFO_STREAM_NAMED(name_, "Displaying Cylinder");
    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    double x_location = 0;
    double y = 0;
    pose1.translation().y() = y;
    double step = 0.025;
    for (double i = 0; i <= 1.0; i += step)
    {
      // publishCylinder(x_pose, RED, length, radius, ns);
      visual_tools_->publishCylinder(pose1, rviz_visual_tools::RAND, 0.01, 0.07);
      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
    }
    visual_tools_->trigger();
    */
    /*
    // Create pose
    Eigen::Isometry3d pose;
    pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
    pose.translation() = Eigen::Vector3d( 0.1, 0.1, 0.1 ); // translate x,y,z

    // Publish arrow vector of pose
    ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
    visual_tools_->publishArrow(pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);

    // Don't forget to trigger the publisher!
    visual_tools_->trigger();
    */
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        visual_tools_->deleteAllMarkers();
        // load constraints for costmap
        double costmap_data[8000];
        for (int i = 0; i < 8000; ++i) costmap_data[ i ] = my_follower.costmap_data[i];//-1000.0;
        for (int i = 0; i < 4000; i ++)
        {
          pose1.translation().x() = costmap_data[i*2+0];
          pose1.translation().y() = costmap_data[i*2+1];
          // publishCylinder(x_pose, RED, length, radius, ns);
          visual_tools_->publishCylinder(pose1, rviz_visual_tools::RED, 0.01, 0.07);
        }
        visual_tools_->trigger();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO_STREAM("Shutting down.");

    return 0;
}
