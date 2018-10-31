#include <tracking/boxes_tracker.h>

BoxesTracker::BoxesTracker():
nh_()
{
	std::string log_output;
	char* envvar_home;
	envvar_home = std::getenv("HOME");
	std::stringstream log_output_aux_stream;
	log_output_aux_stream << envvar_home << "/Matlab_ws/";
	log_output = log_output_aux_stream.str();

	file_merge_box_1_poses.open(log_output + "Test_Pos_Merge_BBXS_Sea/mergeBox1");
  	file_merge_box_2_poses.open(log_output + "Test_Pos_Merge_BBXS_Sea/mergeBox2");
  	// file_merge_box_3_poses.open(log_output + "Test_Pos_Merge_BBXS_Sea/mergeBox3");
 	file_merge_box_1_times.open(log_output + "Test_Pos_Merge_BBXS_Sea/mergeBox1time");
  	file_merge_box_2_times.open(log_output + "Test_Pos_Merge_BBXS_Sea/mergeBox2time");
  	// file_merge_box_3_times.open(log_output + "Test_Pos_Merge_BBXS_Sea/mergeBox3time");

	sub_bounding_box_array = nh_.subscribe("/merge_bounding_boxes", 1, &BoxesTracker::boundingBoxesCallback, this);
	pub_bounding_box_array = nh_.advertise<BoundingBoxArray>("/tracked_bounding_boxes", 1);
	bounding_boxes_past_.header.frame_id = "velodyne";
}

BoxesTracker::~BoxesTracker()
{

}

void BoxesTracker::boundingBoxesCallback(const BoundingBoxArrayConstPtr &input_bounding_box_array)
{
	int i=0, j=0;
	double distance_high_threshold = 7; // 5 meters. Distance limit to match two boxes
	bool past_box_selected = false; // Flag if a matching between a received box and past box has been found
	uint32_t selected_past_label = 0; // Index of the box that match one of the received

	// Traverse received BoundingBoxArray searching for correspondences in the last stored BoundingBoxArray
	ROS_INFO_STREAM("Input array size : " << input_bounding_box_array->boxes.size());
	for(i=0; i < input_bounding_box_array->boxes.size(); i++)
	{
		ROS_INFO_STREAM("Input box : " << i);
		// Set min_distance to infinity
		double min_distance = std::numeric_limits<double>::infinity();
		// Get time stamp of the received box currently being checked for correspondence
		ros::Time input_box_time = input_bounding_box_array->boxes[i].header.stamp;
		ros::Time past_box_time;
		ros::Time aux_time;
		// Get coordinates of each received box
		double x1 = input_bounding_box_array->boxes[i].pose.position.x;
		double y1 = input_bounding_box_array->boxes[i].pose.position.y;
		double z1 =	input_bounding_box_array->boxes[i].pose.position.z;

		// Traverse past BoundingBoxArray
		for(j=0; j < bounding_boxes_past_.boxes.size(); j++)
		{
			// Get coordinates of each stored box
			double x2 = bounding_boxes_past_.boxes[j].pose.position.x;
			double y2 = bounding_boxes_past_.boxes[j].pose.position.y;
			double z2 = bounding_boxes_past_.boxes[j].pose.position.z;
			// Get time stamp of the stored box currently being checked for correspondence
			past_box_time = bounding_boxes_past_.boxes[j].header.stamp;

			// Calculate distance between boxes
			double distance = sqrt(   ((x2 - x1) * (x2 - x1)) 
									+ ((y2 - y1) * (y2 - y1)) 
									+ ((z2 - z1) * (z2 - z1))	);
			// Check if distance is lower than the saved one,lower than the threshold
			// and the difference in observation time is not higher than ... seconds
			ROS_INFO_STREAM("Distance to box " << j << ": " << distance);
			if (   (distance < min_distance ) 
				&& (distance < distance_high_threshold)
				&& ((input_box_time - past_box_time).toSec() <  5) )
			{
				aux_time = past_box_time;
				// Save current calculated distance as the minimum distance between the
				// input box and every stored box
				min_distance = distance;
				// Activate flag to know that a correspondence has been found
				past_box_selected = true;
				// Save the index of the saved box that matches the input box
				selected_past_label = j;
			}
		}
		// If a correspondence has been found, update the information of that box
		if(past_box_selected)
		{
			ROS_INFO_STREAM("Box label :" << selected_past_label);
			ROS_INFO_STREAM("Difference between boxes time stamps: " << input_box_time.toSec() - aux_time.toSec());
			past_box_selected = false;
			bounding_boxes_past_.boxes[selected_past_label] = input_bounding_box_array->boxes[i];
			//bounding_boxes_past_.boxes[selected_past_label].header = input_bounding_box_array->boxes[i].header;
			// Write label according to the index in the vector
			bounding_boxes_past_.boxes[selected_past_label].label = selected_past_label;
		}
		else // If no correspondence has been found, add the box to the end of the array
		{
			int array_size = bounding_boxes_past_.boxes.size();
			bounding_boxes_past_.boxes.push_back(input_bounding_box_array->boxes[i]);
			// Write label according to the index in the vector
			bounding_boxes_past_.boxes.back().label = array_size;

		}
	}

	// Update the time stamp of the bounding box array
	bounding_boxes_past_.header.stamp = ros::Time::now();
	// Publish the new bounding box array in /tracked_bounding_boxes topic
	pub_bounding_box_array.publish(bounding_boxes_past_);

	// Save poses and time of the boxes to plot them in matlab
	if (!flag_tracking_started_)
    {
        time_first_box_received_ = ros::Time::now().toSec();
        flag_tracking_started_ = true;
    }
    for (int i = 0; i < bounding_boxes_past_.boxes.size(); i++)
    {
        switch (bounding_boxes_past_.boxes.size())
        {
        	case 1:
            	file_merge_box_1_poses << bounding_boxes_past_.boxes.at(0).pose.position.x << " " << bounding_boxes_past_.boxes.at(0).pose.position.y << std::endl;
            	file_merge_box_1_times << ros::Time::now().toSec() - time_first_box_received_ << std::endl;
            	break;
        	case 2:
            	file_merge_box_1_poses << bounding_boxes_past_.boxes.at(0).pose.position.x << " " << bounding_boxes_past_.boxes.at(0).pose.position.y << std::endl;
            	file_merge_box_1_times << ros::Time::now().toSec() - time_first_box_received_ << std::endl;
            	file_merge_box_2_poses << bounding_boxes_past_.boxes.at(1).pose.position.x << " " << bounding_boxes_past_.boxes.at(1).pose.position.y << std::endl;
            	file_merge_box_2_times << ros::Time::now().toSec() - time_first_box_received_ << std::endl;
            	break;
        	// case 3:
         //    	file_merge_box_1_poses << bounding_boxes_past_.boxes.at(0).pose.position.x << " " << bounding_boxes_past_.boxes.at(0).pose.position.y << std::endl;
         //    	file_merge_box_1_times << ros::Time::now().toSec() - time_first_box_received_ << std::endl;
         //    	file_merge_box_2_poses << bounding_boxes_past_.boxes.at(1).pose.position.x << " " << bounding_boxes_past_.boxes.at(1).pose.position.y << std::endl;
         //    	file_merge_box_2_times << ros::Time::now().toSec() - time_first_box_received_ << std::endl;
         //    	file_merge_box_3_poses << bounding_boxes_past_.boxes.at(2).pose.position.x << " " << bounding_boxes_past_.boxes.at(2).pose.position.y << std::endl;
         //    	file_merge_box_3_times << ros::Time::now().toSec() - time_first_box_received_ << std::endl;
         //    	break;
        }
    }
}