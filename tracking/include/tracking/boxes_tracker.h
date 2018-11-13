#include <fstream>
#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <geometry_msgs/Point.h>


typedef jsk_recognition_msgs::BoundingBoxArray BoundingBoxArray;
typedef jsk_recognition_msgs::BoundingBox BoundingBox;
typedef boost::shared_ptr<BoundingBoxArray const> BoundingBoxArrayConstPtr;

class BoxesTracker
{
public:
  BoxesTracker();
  ~BoxesTracker();
private:
  // Callback
  void boundingBoxesCallback(const BoundingBoxArrayConstPtr &input_bounding_box_array);

  // Node Handle
  ros::NodeHandle nh_;

  // Subscriber
  ros::Subscriber sub_bounding_box_array;

  // Publisher
  ros::Publisher pub_bounding_box_array;

  // Variables
  BoundingBoxArray bounding_boxes_past_;

  std::ofstream file_merge_box_1_poses;
  std::ofstream file_merge_box_2_poses;
  std::ofstream file_merge_box_3_poses;
  std::ofstream file_merge_box_1_times;
  std::ofstream file_merge_box_2_times;
  std::ofstream file_merge_box_3_times;
  std::ofstream file_merge_box_1_times_abs;
  std::ofstream file_merge_box_2_times_abs;
  std::ofstream file_merge_box_3_times_abs;

  bool flag_tracking_started_; // Flag to signal that the first box has been received
                              // and the time values written to files are calculated from
                              // that moment on
  bool flag_first_box_received_;
  double time_first_box_received_;
};