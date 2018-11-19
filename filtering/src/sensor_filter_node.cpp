#include <filtering/sensor_filter.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_filter_node");

    SensorFilter filter;
    ros::spin();
    return 0;
}