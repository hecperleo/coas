#include <detection/matching.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "matching_node");

    Matching match;

    ros::spin();

    return 0;
}