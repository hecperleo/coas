#include <ros/ros.h>
#include <fstream>
#include <std_msgs/Int8.h>
#include "filtering/VectorInt.h"
#include "filtering/VectorVector.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

typedef std::vector<float> VI;
typedef std::vector<VI> VVI;
typedef std::vector<VVI> VVVI;

class SensorFilter
{

public:
  SensorFilter();
  ~SensorFilter();

private:
  // Callbacks
  void sensorCallback(const sensor_msgs::PointCloud2 &cloud);
  void phaseCallback(const std_msgs::Int8 phase_mode);

  //
  void exploration(const sensor_msgs::PointCloud2 &cloud_3D_usv);
  int isInMap(int i, int j);
  int readV(const VVVI &V, int i, int j);
  //void saveMatrix(const char * file_name, const VVVI &M);
  void saveMatrix3D(const char *file_name, const VVVI &m, bool vel);

  // Node handlers
  ros::NodeHandle n;
  ros::NodeHandle private_nh;

  // Subscribers
  ros::Subscriber sub_phase, sub_sensor;

  // Publishers
  ros::Publisher pub_filtered_cloud_3D, pub_matrix;

  // Variables
  bool use_voxel_filter;
  int phase, range_dock, range_sea, range, rows, columns;
  float cell_div; // number of cell per meter

  // Params
};