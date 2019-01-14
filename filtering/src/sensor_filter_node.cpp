#include <filtering/sensor_filter.h>

int main(int argc, char **argv)
{
    int working_frequency;
    ros::init(argc, argv, "sensor_filter_node");
    ros::NodeHandle pnh("~");
    pnh.param<int>("working_frequency", working_frequency, 10);
    SensorFilter filter;
    ros::Rate rate(working_frequency); //TODO: fine adjustment of working frequency
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}