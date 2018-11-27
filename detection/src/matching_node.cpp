#include <detection/matching.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "matching_node");
    Matching match;
    ros::Rate rate(5); // 5Hz //TODO: fine adjustment of the working frequency
    while(ros::ok())
    {
    	ros::spinOnce();
    	match.toDo();
    	rate.sleep();
    }
    return 0;
}