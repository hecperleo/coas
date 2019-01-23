#include <detection/bounding_boxes.h>

int main(int argc, char **argv)
{
    int working_frequency;
    ros::init(argc, argv, "bounding_boxes_node");
    ros::NodeHandle pnh("~");
    pnh.param<int>("working_frequency", working_frequency, 10);
    BoundingBoxes boundingBoxer;
    ros::Rate rate(working_frequency); //TODO: fine adjustment of working frequency
    while(ros::Time::now() == ros::Time(0)); // If in simulation, wait until /clock messages are published
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}