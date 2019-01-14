#include <detection/matching.h>

int main(int argc, char **argv)
{
    int working_frequency;
    ros::init(argc, argv, "matching_node");
    ros::NodeHandle pnh("~");
    pnh.param<int>("working_frequency", working_frequency, 10);
    Matching match;
    ros::Rate rate(working_frequency); //TODO: fine adjustment of working frequency
    while(ros::ok())
    {
    	ros::spinOnce();
    	match.runOnce();
    	rate.sleep();
    }
    return 0;
}