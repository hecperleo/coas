#ifndef BOUNDING_BOXES_H_
#define BOUNDING_BOXES_H_

#include <cmath>
#include <fstream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <detection/PostsPositions.h>
#include <tracking/LidarCandidatesList.h>
#include <tracking/LidarCandidate.h>

class BoundingBoxes
{

public:
  BoundingBoxes();
  ~BoundingBoxes();

private:
  // Callbacks
  void phaseCallback(const std_msgs::Int8 phaseMode);
  void cloudCallback(const sensor_msgs::PointCloud2Ptr &input_cloud);  
  //
  void getParameters();
  void calculateCenters();
  void cleanVariables();
  void calculateVectorPolygons();
  void mergeBoundingBoxes();
  void savePose(int nPost, nav_msgs::Path path);
  void checkPostDimension(float xDim, float yDim, float zDim);
  void checkDistancesBetweenPosts();
  void calculateMaxDistancesCluster(const pcl::PointCloud<pcl::PointXYZ> cluster);
  float calculateDistance2Points(float x1, float y1, float z1, float x2, float y2, float z2);
  float calculateDistance2Points(const geometry_msgs::Point &point_1, 
                                 const geometry_msgs::Point &point_2);
  void saveDistances(bool b, nav_msgs::Path path1, nav_msgs::Path path2, nav_msgs::Path path3);
  void constructBoundingBoxes(float x, float y, float z, float dimX, float dimY, float dimZ, bool merge);
  nav_msgs::Path constructPath(std::vector<float> x, std::vector<float> y, std::vector<float> z, int length);
  nav_msgs::Path constructPath(const geometry_msgs::Point &point_1, const geometry_msgs::Point &point_2);

  // Node handlers
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Subscribers
  ros::Subscriber sub_filter_points_; // PointCloud subscriber
  ros::Subscriber sub_phase_;
  
  // Cluster Publishers
  ros::Publisher pub_cloud_clusters_;
  // Bounding Boxer Publishers
  ros::Publisher pub_boxes_, pub_merge_boxes_, pub_post_reference_boxes_;
  ros::Publisher pub_path_post_1_, pub_path_post_2_, pub_path_post_3_; 
  ros::Publisher pub_path_post_12_, pub_path_post_13_, pub_path_post_23_;
  ros::Publisher pub_posts_positions_; /// TODO
  // Candidates Publisher
  ros::Publisher pub_lidar_candidates_list_;

  // Variables
  std::string frame_id_;
  int counter_posts_ = 0;
  int label_box_, label_merge_box_;
  std::vector<int> vec_label_polygon_;
  std::vector<std::vector<int>> vec_vec_label_polygon_;
  jsk_recognition_msgs::BoundingBox box_;
  jsk_recognition_msgs::BoundingBoxArray boxes_, post_candidates_boxes_, post_reference_boxes_ ,merge_boxes_;
  float x_max_, y_max_, z_max_;
  float x_min_, y_min_, z_min_;
  float x_center_, y_center_, z_center_;
  float max_dist_x_, max_dist_y_, max_dist_z_;
  float max_dist_x_polygon_, max_dist_y_polygon_, max_dist_z_polygon_;
  nav_msgs::Path path_post_1_, path_post_2_, path_post_3_, path_post_12_, path_post_13_, path_post_23_;

  std::string log_output_;
  int contTest_, contTestPose_;
  double time_start_, time_pose_start_;
  std::ofstream file_distance_to_posts_, file_distance_between_posts_; 
  std::ofstream file_distance_to_posts_times_, file_distance_between_posts_times_;
  std::ofstream file_post_1_, file_post_2_, file_post_3_;
  std::ofstream file_post_1_time_, file_post_2_time_, file_post_3_time_;

  // Phase
  int phase_; // 1 Docking - 2 Harbor - 3 Sea

  // DOCKING parameters
  float docking_distance_threshold_, docking_cluster_tolerance_;
  int docking_min_cluster_size_, docking_max_cluster_size_;
  float docking_close_distance_, docking_xy_min_post_, docking_xy_max_post_, docking_z_min_post_, docking_z_max_post_;
  float docking_min_distance_post_12_, docking_max_distance_post_12_;
  float docking_min_distance_post_13_, docking_max_distance_post_13_;
  // HARBOR parameters
  float harbor_distance_threshold_, harbor_cluster_tolerance_;
  int harbor_min_cluster_size_, harbor_max_cluster_size_;
  float harbor_close_distance_, harbor_xy_min_post_, harbor_xy_max_post_, harbor_z_min_post_, harbor_z_max_post_;
  float harbor_min_distance_post_12_, harbor_max_distance_post_12_;
  float harbor_min_distance_post_13_, harbor_max_distance_post_13_;
  // SEA parameters
  float sea_distance_threshold_, sea_cluster_tolerance_;
  int sea_min_cluster_size_, sea_max_cluster_size_;
  float sea_close_distance_, sea_xy_min_post_, sea_xy_max_post_, sea_z_min_post_, sea_z_max_post_;
  float sea_min_distance_post_12_, sea_max_distance_post_12_;
  float sea_min_distance_post_13_, sea_max_distance_post_13_;

  // Euclidean Clusterer specific parameters
  float distance_threshold_, cluster_tolerance_, min_cluster_size_, max_cluster_size_;
  // Bounding Boxes specific parameters
  float close_distance_, xy_min_post_, xy_max_post_, z_min_post_, z_max_post_;
  float min_distance_post_12_, max_distance_post_12_;
  float min_distance_post_13_, max_distance_post_13_;
};

#endif