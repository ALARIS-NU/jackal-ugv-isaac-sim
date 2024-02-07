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
ros::Publisher pub1;
ros::Publisher pub_coordinates;

//int goal_pixel_bounds[] = {-1, -1, -1, -1};
int goal_pixel[] = {-1, -1};

void mapping(float x, float y, float z, int pixel_X_Y[2]){
    // INTRINSICS DEPTH
    int width = 640;
    int height = 480;
    int ppx = 320;
    int ppy = 240;
    float fx = 462.1379699707031;
    float fy = 462.1379699707031;
    float coeffs[] = {0.0, 0.0, 0.0, 0.0, 0.0};

    float float_x = (x)/z;
    //float float_x = (x)/z;
    float float_y = y/z;

    float float_r2 = float_x * float_x + float_y * float_y;
    float float_f = 1.0 + coeffs[0] * float_r2 + coeffs[1] * float_r2 * float_r2 + coeffs[4]* float_r2 * float_r2* float_r2;
    float_x *= float_f;
    float_y *= float_f;

    float float_dx = float_x + 2.0 * coeffs[2] * float_x * float_y + coeffs[3] * (float_r2 + 2 * float_x * float_x);
    float float_dy = float_y + 2.0 * coeffs[3] * float_x * float_y + coeffs[2] * (float_r2 + 2 * float_y * float_y);
    float_x = float_dx;
    float_y = float_dy;

    pixel_X_Y[0] = int(float_x * fx + ppx);
    pixel_X_Y[1] = int(float_y * fy + ppy);

    return;
}

void selector_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)  {
    goal_pixel[0] = msg->data[0];
    goal_pixel[1] = msg->data[1];
    //goal_pixel_bounds[2] = msg->data[2];
    //goal_pixel_bounds[3] = msg->data[3];
}


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
  transform.setRotation(q);
  
  ros::WallTime start_, end_;
  double execution_time = 0.0;
  //ROS_INFO("Got callback\n");
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);
  //ROS_INFO("Initial points %i \n",  cloud->points.size());
  //pcl::ModelCoefficients coefficients;
  //pcl::PointIndices inliers;

  // Statistical outlier
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (20);
  sor.setStddevMulThresh (0.1);
  sor.filter(*cloud_filtered);
  
  Eigen::Vector4f centroid; 
  //pcl::compute3DCentroid(*cloud_filtered, centroid);
  //transform.setOrigin( tf::Vector3(centroid[0], centroid[1], centroid[2]) );
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "outlier_removal"));

  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZ> sor2;
  sor2.setInputCloud (cloud_filtered);
  sor2.setLeafSize (0.01f, 0.01f, 0.01f);
  sor2.filter (*cloud_filtered1);
  cloud_filtered.swap(cloud_filtered1) ;

  //pcl::compute3DCentroid(*cloud_filtered, centroid);
  //transform.setOrigin( tf::Vector3(centroid[0], centroid[1], centroid[2]) );
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "downsampling"));

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered1);

  //pcl::compute3DCentroid(*cloud_filtered1, centroid);
  //transform.setOrigin( tf::Vector3(centroid[0], centroid[1], centroid[2]) );
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "filtering"));

  //ROS_INFO("Filtered\n");
  
  start_ = ros::WallTime::now();
  // Plane segmentaiton
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  
  // remove all planes
  int count=0;
  int num_of_points = cloud_filtered1->points.size();
  int current_points = num_of_points;
  while (current_points > 0.0700*num_of_points) {
      seg.setInputCloud (cloud_filtered1);
      seg.segment (*inliers, *coefficients);
      
      if (inliers->indices.size () == 0)
      {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        break;
      }

      //ROS_INFO("Segmentation Done\n");

      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud_filtered1);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (*cloud_p);
      current_points = cloud_p->points.size();
      cloud_filtered1.swap(cloud_p) ;
      count++;
      // Note! - record the data of planes to allow object touching
      //ROS_INFO("Final points %i / %i \n",  current_points, num_of_points);
      //ROS_INFO("After swap %i / %i \n",  cloud_p->points.size(), cloud_filtered1->points.size());
  }
  end_ = ros::WallTime::now();
  // print results
  execution_time = (end_ - start_).toNSec() * 1e-6;
  //ROS_INFO("Segmentation time (ms): %f", execution_time);
  //ROS_INFO("Final points %i / %i ",  cloud_filtered1->points.size(), num_of_points);
  
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 cloud_inliers_pcl2;
  pcl::toPCLPointCloud2(*cloud_filtered1, cloud_inliers_pcl2);
  pcl_conversions::fromPCL(cloud_inliers_pcl2, output);

  pub.publish (output);

  pcl::compute3DCentroid(*cloud_filtered1, centroid);
  transform.setOrigin( tf::Vector3(centroid[0], centroid[1], centroid[2]) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "no_planes"));

  /*if (goal_pixel_bounds[0]<0 || goal_pixel_bounds[1]<0 || goal_pixel_bounds[2]<0 || goal_pixel_bounds[3]<0) {
      ROS_INFO("No object selected!");
      return;
  }*/
  if (goal_pixel[0]<0 || goal_pixel[1]<0) {
      ROS_INFO("No object selected!");
      return;
  }



  // Start KdTree
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
    
    // Compute centroid for claster and publish TF
    //transform.setOrigin( tf::Vector3(my_point_x/my_point_count, my_point_y/my_point_count, my_point_z/my_point_count) );
    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", std::to_string(size_by_hand)));
    //ROS_INFO("cluster: %i", size_by_hand);

    // Compute Axis Aligned Bounding Box
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_cluster);
    feature_extractor.compute();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB(min_point_AABB,max_point_AABB);
    /*
    std::string s_min = "min";
    std::string s_max = "max";

    transform.setOrigin( tf::Vector3(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", s_min+std::to_string(size_by_hand)));
    
    transform.setOrigin( tf::Vector3(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", s_max+std::to_string(size_by_hand)));
    */
    int centroid_pixel_X_Y[] = {0, 0};
    int min_pixel_X_Y[] = {0, 0};
    int max_pixel_X_Y[] = {0, 0};
    mapping( my_point_x/my_point_count, my_point_y/my_point_count, my_point_z/my_point_count, centroid_pixel_X_Y);
    mapping( min_point_AABB.x, min_point_AABB.y, min_point_AABB.z, min_pixel_X_Y);
    mapping( max_point_AABB.x, max_point_AABB.y, max_point_AABB.z, max_pixel_X_Y);

    //float temp_dist = (goal_pixel_bounds[0] - min_pixel_X_Y[0])*(goal_pixel_bounds[0] - min_pixel_X_Y[0]) +  (goal_pixel_bounds[1] - min_pixel_X_Y[1])*(goal_pixel_bounds[1] - min_pixel_X_Y[1])
    //                 +(goal_pixel_bounds[2] - max_pixel_X_Y[0])*(goal_pixel_bounds[2] - max_pixel_X_Y[0]) +  (goal_pixel_bounds[3] - max_pixel_X_Y[1])*(goal_pixel_bounds[3] - max_pixel_X_Y[1]);

    float temp_dist = (goal_pixel[0] - centroid_pixel_X_Y[0])*(goal_pixel[0] - centroid_pixel_X_Y[0]);

    if (temp_dist<min_2d_dist) {
        min_2d_dist = temp_dist;
        selected_cluster = size_by_hand; // set current cluster as selected
    }
    ROS_INFO("%i xyz %f %f %f center %i %i", size_by_hand, my_point_x/my_point_count, my_point_y/my_point_count, my_point_z/my_point_count, centroid_pixel_X_Y[0], centroid_pixel_X_Y[1]);

    size_by_hand++;
  }
  ROS_INFO("Number of clusters: %i, Selected cluster %i", cluster_indices.size(), selected_cluster);

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


  transform.setOrigin( tf::Vector3(my_point_x/my_point_count, my_point_y/my_point_count, my_point_z/my_point_count) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "Selected_object"));
  transform.setOrigin( tf::Vector3(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "Selecteed_BB_min"));
  transform.setOrigin( tf::Vector3(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "Selecteed_BB_max"));

  std_msgs::Float64MultiArray selected_object_coordinates;
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

  // Publish the data
  output2.header.frame_id = "camera_depth_optical_frame";
  pub1.publish (output2);

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
  ros::Subscriber sub = nh.subscribe ("/selected_objec_2d", 1, selector_cb);
  ros::Subscriber sub_selector = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);
  //ros::Subscriber sub = nh.subscribe ("kinect/depth_registered/points", 1, cloud_cb);
  //ROS_INFO("Sub_started\n");
  // Create a ROS publisher for the output model coefficients
  //pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output_cloud", 1);

  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output_clusters", 1);
  
  pub_coordinates = nh.advertise<std_msgs::Float64MultiArray> ("selected_object_coordinates", 1);
  // Spin
  ros::spin ();
}
