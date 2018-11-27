#ifndef MATCHING_H_
#define MATCHING_H_

#include <fstream>
#include <Eigen/Eigen>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include "visualization_msgs/Marker.h"

#include <detection/PostsPositions.h>

typedef std::pair<int, float> pair_int_float;

class Matching
{

public:
  Matching();
  ~Matching();

  void toDo();

private:
  //
  void drawPosts();
  void savePose(const int &nPost, const geometry_msgs::Point &waypoint);
  float calculateDistance2Points(const float &x1, const float &y1, const float &z1, 
                                 const float &x2, const float &y2, const float &z2);
  float calculateDistance2Points(const geometry_msgs::Point &point_1, 
                                 const geometry_msgs::Point &point_2);
  // Callbacks
  void postsPositionsCallback(const detection::PostsPositions &posts_positions); //// TODO
  void calculateRotationMatrix(const float &angle, Eigen::Matrix2f &R); /// TODO
  void calculateCentroid3Points2D(Eigen::Vector2f &centroid, 
                                            const geometry_msgs::Point &point_1, 
                                            const geometry_msgs::Point &point_2,
                                            const geometry_msgs::Point &point_3); /// TODO
  // Node handlers
  ros::NodeHandle nh_, pnh_;

  // Subscribers
  ros::Subscriber sub_posts_positions_;

  // Publishers
  ros::Publisher pub_marker_1_, pub_marker_2_, pub_marker_3_, pub_marker_4_, pub_marker_5_, pub_marker_6_;
  ros::Publisher pub_position_post_1_, pub_position_post_2_, pub_position_post_3_;

  // Variables
  std::string frame_id_;
  int counter_;
  geometry_msgs::Point now_position_post_1_, now_position_post_2_, now_position_post_3_;
  std::vector<geometry_msgs::Point> prev_positions_posts_, now_positions_posts_;
  std::ofstream file_post_1_, file_post_2_, file_post_3_, file_post_1_time_, file_post_2_time_, file_post_3_time_;
  double start_time_pose_;
  // Markers
  visualization_msgs::Marker marker_post_1_, marker_post_2_, marker_post_3_; 
  visualization_msgs::Marker marker_post_4_, marker_post_5_, marker_post_6_;

  bool flag_position_post_1_;
  bool flag_position_post_2_;
  bool flag_position_post_3_;

  // VIRTUAL POST CREATION DEBUGGING FILES
  std::ofstream file_virtual_post_1_, file_virtual_post_2_, file_virtual_post_3_;
};

#endif