#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/ml/kmeans.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <string> 

ros::Publisher pub;
ros::Publisher pub_coordinates;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  ROS_INFO("Inside subscriber!");
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
  transform.setRotation(q);
  
  ros::WallTime start_, end_;
  double execution_time = 0.0;
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
  
  start_ = ros::WallTime::now();
  // Convert message to point cloud PCL format
  pcl::fromROSMsg (*input, *cloud);

  /*
  // Statistical outlier
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (20);
  sor.setStddevMulThresh (0.1);
  sor.filter(*cloud_filtered);
  */
  
  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZ> sor2;
  sor2.setInputCloud (cloud);
  sor2.setLeafSize (0.10f, 0.10f, 0.10f);
  sor2.filter(*cloud_filtered1);
  cloud_filtered.swap(cloud_filtered1);

  // Create the filtering object   <- here we can set the detection region boundaries 
  /*pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered1);*/
  
  // Convert to ROS data type to visualize filtering
  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 cloud_inliers_pcl2;
  pcl::toPCLPointCloud2(*cloud_filtered1, cloud_inliers_pcl2);
  pcl_conversions::fromPCL(cloud_inliers_pcl2, output);
  output.header.frame_id = "camera";
  pub.publish(output);
  
  end_ = ros::WallTime::now();
  execution_time = (end_ - start_).toNSec() * 1e-6;
  ROS_INFO("Segmentation time (ms): %f", execution_time);

  /*
  // Start KdTree to identify the clasters of points
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered1);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.05); // 1cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (2000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered1);
  ec.extract (cluster_indices);
  //std::cout << cluster_indices.empty() << " " << cluster_indices.size() << "\n";
  if (cluster_indices.empty()) {
      ROS_INFO("No clusters!");
      return;
  }
  */
  /*
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  int selected_cluster = 0;
  float min_2d_dist = 1000000000000.0;
  int size_by_hand = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    int my_point_count = 0;
    float my_point_x = 0;
    float my_point_y = 0;
    float my_point_z = 0;
    cloud_cluster->clear();
    for (const auto& idx : it->indices) {
        cloud_cluster->push_back ((*cloud_filtered1)[idx]);
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        my_point_x += (*cloud_filtered1)[idx].x;
        my_point_y += (*cloud_filtered1)[idx].y;
        my_point_z += (*cloud_filtered1)[idx].z;
        my_point_count++;
    }
    
    // Compute Axis Aligned Bounding Box
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_cluster);
    feature_extractor.compute();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB(min_point_AABB,max_point_AABB);
    
    int centroid_pixel_X_Y[] = {0, 0};
    int min_pixel_X_Y[] = {0, 0};
    int max_pixel_X_Y[] = {0, 0};


    size_by_hand++;
  }
  ROS_INFO("Number of clusters: %i, Selected cluster %i", cluster_indices.size(), selected_cluster);
  */
  /*
  int my_point_count = 0;
  float my_point_x = 0;
  float my_point_y = 0;
  float my_point_z = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& idx : cluster_indices.at(selected_cluster).indices) {
      final_cluster->push_back ((*cloud_filtered1)[idx]);
      my_point_x += (*cloud_filtered1)[idx].x;
      my_point_y += (*cloud_filtered1)[idx].y;
      my_point_z += (*cloud_filtered1)[idx].z;
      my_point_count++;
  }
  final_cluster->width = final_cluster->size ();
  final_cluster->height = 1;
  final_cluster->is_dense = true;
  
  start_ = ros::WallTime::now();
  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(final_cluster);
  feature_extractor.compute();
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  feature_extractor.getAABB(min_point_AABB,max_point_AABB);
  
  sensor_msgs::PointCloud2 output2;
  pcl::PCLPointCloud2 cloud_inliers_pcl2_2;
  pcl::toPCLPointCloud2(*final_cluster, cloud_inliers_pcl2_2);
  pcl_conversions::fromPCL(cloud_inliers_pcl2_2, output2);
  */

  /*transform.setOrigin( tf::Vector3(my_point_x/my_point_count, my_point_y/my_point_count, my_point_z/my_point_count) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "Selected_object"));
  transform.setOrigin( tf::Vector3(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "Selecteed_BB_min"));
  transform.setOrigin( tf::Vector3(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "Selecteed_BB_max"));*/

  // publish coordinates and BB or radius
  /*std_msgs::Float64MultiArray selected_object_coordinates;
  selected_object_coordinates.data.clear();
  selected_object_coordinates.data.push_back(my_point_x/my_point_count);
  selected_object_coordinates.data.push_back(my_point_y/my_point_count);
  selected_object_coordinates.data.push_back(my_point_z/my_point_count);
  selected_object_coordinates.data.push_back(min_point_AABB.x);
  selected_object_coordinates.data.push_back(min_point_AABB.y);
  selected_object_coordinates.data.push_back(min_point_AABB.z);
  selected_object_coordinates.data.push_back(max_point_AABB.x);
  selected_object_coordinates.data.push_back(max_point_AABB.y);
  selected_object_coordinates.data.push_back(max_point_AABB.z);
  pub_coordinates.publish(selected_object_coordinates);
  */

  // Publish the data
  //output2.header.frame_id = "camera_depth_optical_frame";
  //pub1.publish (output2);

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ros::Rate rate(0.1); // frequency of operation
  ROS_INFO("Node_started\n");
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);

  pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output_cloud", 1);
  
  pub_coordinates = nh.advertise<std_msgs::Float64MultiArray> ("selected_object_coordinates", 1);
  // Spin
  ROS_INFO("Start spinning!");
  ros::spin ();
}
