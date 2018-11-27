#include <detection/matching.h>

Matching::Matching(): nh_(), pnh_("~")
{
    // Params
    pnh_.param<std::string>("frame_id", frame_id_, "/velodyne");

    // Subscriptions
    sub_path_post_1_ = nh_.subscribe("/path_post_1", 1, &Matching::pathPost1Callback, this);
    sub_path_post_2_ = nh_.subscribe("/path_post_2", 1, &Matching::pathPost2Callback, this);
    sub_path_post_3_ = nh_.subscribe("/path_post_3", 1, &Matching::pathPost3Callback, this);
    sub_path_post_12_ = nh_.subscribe("/path_post_12", 1, &Matching::pathPost12Callback, this);
    sub_path_post_13_ = nh_.subscribe("/path_post_13", 1, &Matching::pathPost13Callback, this);
    sub_path_post_23_ = nh_.subscribe("/path_post_23", 1, &Matching::pathPost23Callback, this);

    // Publishers
    pub_marker_1_ = nh_.advertise<visualization_msgs::Marker>("/marker_post_1", 1);
    pub_marker_2_ = nh_.advertise<visualization_msgs::Marker>("/marker_post_2", 1);
    pub_marker_3_ = nh_.advertise<visualization_msgs::Marker>("/marker_post_3", 1);
    pub_marker_4_ = nh_.advertise<visualization_msgs::Marker>("/marker_post_4", 1);
    pub_marker_5_ = nh_.advertise<visualization_msgs::Marker>("/marker_post_5", 1);
    pub_marker_6_ = nh_.advertise<visualization_msgs::Marker>("/marker_post_6", 1);

    char *envvar_home;
    envvar_home = std::getenv("HOME");
    std::stringstream log_output_aux;
    log_output_aux << envvar_home << "/Matlab_ws/";
    std::string log_output;
    log_output = log_output_aux.str();

    file_post_1_.open(log_output + "matchPost1");
    file_post_2_.open(log_output + "matchPost2");
    file_post_3_.open(log_output + "matchPost3");
    file_post_1_time_.open(log_output + "matchPost1time");
    file_post_2_time_.open(log_output + "matchPost2time");
    file_post_3_time_.open(log_output + "matchPost3time");

    counter_ = 0;

    // INITIALIZE MARKERS WITH DEFAULT VALUES THAT DOESN'T CHANGE DURING EXECUTION
    marker_post_1_.header.frame_id = marker_post_2_.header.frame_id = marker_post_3_.header.frame_id = "/velodyne";
    marker_post_4_.header.frame_id = marker_post_5_.header.frame_id = marker_post_6_.header.frame_id = "/velodyne";
    marker_post_1_.header.stamp = marker_post_2_.header.stamp = marker_post_3_.header.stamp = ros::Time();
    marker_post_4_.header.stamp = marker_post_5_.header.stamp = marker_post_6_.header.stamp = ros::Time();

    geometry_msgs::Quaternion orientation;
    orientation.x = orientation.y = orientation.z = 0.0;
    orientation.w = 1.0;
    marker_post_1_.pose.orientation = marker_post_2_.pose.orientation = marker_post_3_.pose.orientation = orientation;
    marker_post_4_.pose.orientation = marker_post_5_.pose.orientation = marker_post_6_.pose.orientation = orientation;
    marker_post_1_.ns = marker_post_2_.ns = marker_post_3_.ns = "my_namespace";
    marker_post_4_.ns = marker_post_5_.ns = marker_post_6_.ns = "my_namespace";

    marker_post_1_.id = 1;
    marker_post_2_.id = 2;
    marker_post_3_.id = 3;
    marker_post_4_.id = 4;
    marker_post_5_.id = 5;
    marker_post_6_.id = 6;

    marker_post_1_.type = marker_post_2_.type = marker_post_3_.type = visualization_msgs::Marker::CYLINDER;
    marker_post_4_.type = marker_post_5_.type = marker_post_6_.type = visualization_msgs::Marker::CYLINDER;
    marker_post_1_.action = marker_post_2_.action = marker_post_3_.action = visualization_msgs::Marker::ADD;
    marker_post_4_.action = marker_post_5_.action = marker_post_6_.action = visualization_msgs::Marker::ADD;

    marker_post_1_.lifetime = marker_post_2_.lifetime = marker_post_3_.lifetime = ros::Duration(0.066);
    marker_post_4_.lifetime = marker_post_5_.lifetime = marker_post_6_.lifetime = ros::Duration(0.066);

    geometry_msgs::Vector3 scale;
    scale.x = scale.y = scale.z = 0.6;
    marker_post_1_.scale = marker_post_2_.scale = marker_post_3_.scale = scale;
    scale.x = scale.y = 0.3;
    scale.z = 1.0;
    marker_post_4_.scale = marker_post_5_.scale = marker_post_6_.scale = scale;

    marker_post_1_.color.a = 0.8;
    marker_post_1_.color.r = 1.0;
    marker_post_1_.color.g = marker_post_1_.color.b = 0.0;

    marker_post_2_.color.a = 0.8;
    marker_post_2_.color.r = marker_post_2_.color.b = 0.0;
    marker_post_2_.color.g = 1.0;

    marker_post_3_.color.a = 0.8;
    marker_post_3_.color.r = marker_post_3_.color.g = 0.0;
    marker_post_3_.color.b = 1.0;

    marker_post_4_.color.a = 1.0;
    marker_post_4_.color.r = marker_post_4_.color.g = 0.5;
    marker_post_4_.color.b = 0.0;

    marker_post_5_.color.a = 1.0;
    marker_post_5_.color.r = marker_post_5_.color.b = 0.5;
    marker_post_5_.color.g = 0.0;

    marker_post_6_.color.a = 1.0;
    marker_post_6_.color.r = marker_post_6_.color.b = 0.0;
    marker_post_6_.color.g = 0.5;
    // END MARKERS INITIALIZATION ///////////////

    toDo();
}

Matching::~Matching()
{
    file_post_1_.close();
    file_post_2_.close();
    file_post_3_.close();
    file_post_1_time_.close();
    file_post_2_time_.close();
    file_post_3_time_.close();
}

void Matching::pathPost1Callback(const nav_msgs::Path &path)
{
    now_path_post_1_ = path;
}

void Matching::pathPost2Callback(const nav_msgs::Path &path)
{
    now_path_post_2_ = path;
}

void Matching::pathPost3Callback(const nav_msgs::Path &path)
{
    now_path_post_3_ = path;
}

void Matching::pathPost12Callback(const nav_msgs::Path &path)
{
    now_path_post_12_ = path;
    toDo();
}

void Matching::pathPost13Callback(const nav_msgs::Path &path)
{
    now_path_post_13_ = path;
    toDo();
}

void Matching::pathPost23Callback(const nav_msgs::Path &path)
{

    now_path_post_23_ = path;
    toDo();
}

float Matching::calculateDistance2Points(const float &x1, const float &y1, const float &z1, 
                                         const float &x2, const float &y2, const float &z2)
{
    float distance;
    distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1)));
    return distance;
}

void Matching::drawPosts()
{
    switch(prev_path_posts_.poses.size())
    {
        case 3:
            marker_post_3_.pose.position = prev_path_posts_.poses.at(2).pose.position;
            pub_marker_3_.publish(marker_post_3_);
        case 2:
            marker_post_2_.pose.position = prev_path_posts_.poses.at(1).pose.position;
            pub_marker_2_.publish(marker_post_2_);
        case 1:
            marker_post_1_.pose.position = prev_path_posts_.poses.at(0).pose.position;
            pub_marker_1_.publish(marker_post_1_);
            break;
    }
    marker_post_6_.pose.position = now_path_posts_.poses.at(2).pose.position;
    pub_marker_6_.publish(marker_post_6_);
    marker_post_5_.pose.position = now_path_posts_.poses.at(1).pose.position;
    pub_marker_5_.publish(marker_post_5_);
    marker_post_4_.pose.position = now_path_posts_.poses.at(0).pose.position;
    pub_marker_4_.publish(marker_post_4_);
}

void Matching::savePose(int &nPost, geometry_msgs::PoseStamped &waypoint)
{
    switch (nPost)
    {
    case 0:
        file_post_1_ << waypoint.pose.position.x << " " << waypoint.pose.position.y << std::endl;
        file_post_1_time_ << ros::Time::now().toSec() - start_time_pose_ << std::endl;
        break;
    case 1:
        file_post_2_ << waypoint.pose.position.x << " " << waypoint.pose.position.y << std::endl;
        file_post_2_time_ << ros::Time::now().toSec() - start_time_pose_ << std::endl;
        break;
    case 2:
        file_post_3_ << waypoint.pose.position.x << " " << waypoint.pose.position.y << std::endl;
        file_post_3_time_ << ros::Time::now().toSec() - start_time_pose_ << std::endl;
        break;
    }
}

void Matching::toDo()
{
    // At the initial instant it stores the first entry as valid labels
    switch (counter_)
    {
    case 0:
        // It initializes if there are three valid posts
        if (!now_path_post_12_.poses.empty() && !now_path_post_13_.poses.empty() && !now_path_post_23_.poses.empty())
        {
            ROS_WARN("Init Matching");
            // It storages the three relevant waypoints of the actual state into the previous state vector
            prev_path_posts_.poses.push_back(now_path_post_1_.poses.at(1));
            prev_path_posts_.poses.push_back(now_path_post_2_.poses.at(1));
            prev_path_posts_.poses.push_back(now_path_post_3_.poses.at(1));
            counter_++;
            start_time_pose_ = ros::Time::now().toSec();
        }
        break;
    case 1:
        // If there are three valid posts and paths did not change
        if ((!now_path_post_12_.poses.empty() && (!now_path_post_13_.poses.empty() || !now_path_post_23_.poses.empty())) ||
            (!now_path_post_13_.poses.empty() && (!now_path_post_12_.poses.empty() || !now_path_post_23_.poses.empty())) ||
            (!now_path_post_23_.poses.empty() && (!now_path_post_12_.poses.empty() || !now_path_post_13_.poses.empty())) )
        {
            // Store the six relevant waypoints of the actual state in a vector
            if (!now_path_post_12_.poses.empty())
            {
                now_path_posts_.poses.push_back(now_path_post_12_.poses.at(0));
                now_path_posts_.poses.push_back(now_path_post_12_.poses.at(1));
            }
            if (!now_path_post_13_.poses.empty())
            {
                now_path_posts_.poses.push_back(now_path_post_13_.poses.at(0));
                now_path_posts_.poses.push_back(now_path_post_13_.poses.at(1));
            }
            if (!now_path_post_23_.poses.empty())
            {
                now_path_posts_.poses.push_back(now_path_post_23_.poses.at(0));
                now_path_posts_.poses.push_back(now_path_post_23_.poses.at(1));
            }
            std::vector<int> vec_labels;
            std::vector<float> vec_check_dist;
            std::map<int, float> map_distance_and_label;
            // Compare each relevant waypoint of the previous state with the waypoints of the actual state
            for (int i = 0; i < prev_path_posts_.poses.size(); i++)
            {
                for (int j = 0; j < now_path_posts_.poses.size(); j++)
                {
                    geometry_msgs::Point position_aux_1 = prev_path_posts_.poses.at(i).pose.position;
                    geometry_msgs::Point position_aux_2 = now_path_posts_.poses.at(j).pose.position;
                    // Calculate the distance between the waypoints of the previous state and the waypoints of the current state
                    map_distance_and_label[j] = calculateDistance2Points(position_aux_1.x, position_aux_1.y, position_aux_1.z,
                                                                         position_aux_2.x, position_aux_2.y, position_aux_2.z);
                }

                // Sort all distances from lowest to highest
                std::vector<pair_int_float> vec;
                std::copy(map_distance_and_label.begin(), map_distance_and_label.end(), std::back_inserter<std::vector<pair_int_float>>(vec));
                std::sort(vec.begin(), vec.end(),
                          [](const pair_int_float &l, const pair_int_float &r) {
                              if (l.second != r.second)
                                  return l.second < r.second;
                              return l.first < r.first;
                          });

                vec_labels.push_back(vec[0].first);
                vec_check_dist.push_back(vec[0].second);
            }
            // Clean the vector of waypoints from the previous state
            prev_path_posts_.poses.clear();
            prev_path_posts_.header.frame_id = "velodyne";
            // [LOOP .BAG]
            // for (int i = 0; i < vec_check_dist.size(); i++)
            // {
            //     if (vec_check_dist[i] > 2.0)
            //     {
            //         counter = 0;
            //         ROS_WARN("Reset Matching");
            //         break;
            //     }
            // }
            // // [LOOP.BAG]
            // if (counter == 0)
            // {
            //     break;
            // }
            // Storage as previous state the posts with the correct label placed
            for (int i = 0; i < vec_labels.size(); i++)
            {
                prev_path_posts_.poses.push_back(now_path_posts_.poses.at(vec_labels[i]));
            }
            // Storage identified posts positions
            for (int i = 0; i < prev_path_posts_.poses.size(); i++)
            {
                savePose(i, prev_path_posts_.poses.at(i));
            }
            // Use markers to visualize on Rviz the result
            drawPosts();
            // Clean vectors
            vec_labels.clear();
            vec_check_dist.clear();
            map_distance_and_label.clear();
            now_path_posts_.poses.clear();
            now_path_post_23_.poses.clear();
        }
        break;
    }
}
