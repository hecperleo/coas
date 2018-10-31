#include <tracking/boxes_tracker.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "boxes_tracker_node");
	BoxesTracker boxes_tracker;
	ros::spin();
	return 0;
}