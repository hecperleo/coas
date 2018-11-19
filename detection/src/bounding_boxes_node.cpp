#include <detection/bounding_boxes.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bounding_boxes_node");

    BoundingBoxes boundingBoxer;

    ros::spin();

    return 0;
}