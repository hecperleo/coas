#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "visualization_msgs/Marker.h"

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
  void savePose(int &nPost, geometry_msgs::PoseStamped &waypoint);
  float calculateDistance2Points(const float &x1, const float &y1, const float &z1, 
                                 const float &x2, const float &y2, const float &z2);

  // Callbacks
  void pathPost1Callback(const nav_msgs::Path &path);
  void pathPost2Callback(const nav_msgs::Path &path);
  void pathPost3Callback(const nav_msgs::Path &path);
  void pathPost12Callback(const nav_msgs::Path &path);
  void pathPost13Callback(const nav_msgs::Path &path);
  void pathPost23Callback(const nav_msgs::Path &path);

  // Node handlers
  ros::NodeHandle nh_, pnh_;

  // Subscribers
  ros::Subscriber sub_path_post_1_, sub_path_post_2_, sub_path_post_3_;
  ros::Subscriber sub_path_post_12_, sub_path_post_13_, sub_path_post_23_;

  // Publishers
  ros::Publisher pub_marker_1_, pub_marker_2_, pub_marker_3_, pub_marker_4_, pub_marker_5_, pub_marker_6_;

  // Variables
  std::string frame_id_;
  int counter_;
  nav_msgs::Path now_path_post_1_, now_path_post_2_, now_path_post_3_;
  nav_msgs::Path now_path_post_12_, now_path_post_13_, now_path_post_23_;
  nav_msgs::Path prev_path_posts_, now_path_posts_;
  std::ofstream file_post_1_, file_post_2_, file_post_3_, file_post_1_time_, file_post_2_time_, file_post_3_time_;

  double start_time_pose_;
  // Markers
  visualization_msgs::Marker marker_post_1_, marker_post_2_, marker_post_3_;
  visualization_msgs::Marker marker_post_4_, marker_post_5_, marker_post_6_;
};