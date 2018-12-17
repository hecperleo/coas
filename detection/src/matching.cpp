#include <detection/matching.h>
typedef std::pair<int,float> pair_int_float;

Matching::Matching(): nh_(), pnh_("~")
{
    // Params
    pnh_.param<std::string>("frame_id", frame_id_, "/velodyne");
    pnh_.param<bool>("generate_virtual_post", flag_gen_virtual_post_, false);

    // Subscriptions
    sub_posts_positions_ = nh_.subscribe("posts_positions", 1, &Matching::postsPositionsCallback, this); ///// TODO

    // Publishers
    pub_marker_1_ = nh_.advertise<visualization_msgs::Marker>("marker_post_1", 1);
    pub_marker_2_ = nh_.advertise<visualization_msgs::Marker>("marker_post_2", 1);
    pub_marker_3_ = nh_.advertise<visualization_msgs::Marker>("marker_post_3", 1);
    pub_marker_4_ = nh_.advertise<visualization_msgs::Marker>("marker_post_4", 1);
    pub_marker_5_ = nh_.advertise<visualization_msgs::Marker>("marker_post_5", 1);
    pub_marker_6_ = nh_.advertise<visualization_msgs::Marker>("marker_post_6", 1);

    pub_position_post_1_ = nh_.advertise<geometry_msgs::PointStamped>("position_post_1", 1);
    pub_position_post_2_ = nh_.advertise<geometry_msgs::PointStamped>("position_post_2", 1);
    pub_position_post_3_ = nh_.advertise<geometry_msgs::PointStamped>("position_post_3", 1);
    flag_position_post_1_ = false;
    flag_position_post_2_ = false;
    flag_position_post_3_ = false;

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

    /// VIRTUAL POST CREATION DEBUGGING FILES
    std::stringstream log_virtual_post;
    log_virtual_post << envvar_home << "/ros_log/";
    //file_virtual_post_1_.open(log_virtual_post + "virtual_post_1");
    //file_virtual_post_2_.open(log_virtual_post + "virtual_post_2");
    file_virtual_post_3_.open(log_virtual_post.str() + "virtual_post_3");

    flag_matching_initialized_ = false;

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

void Matching::postsPositionsCallback(const detection::PostsPositions &posts_positions)
{
    //std::cout << "Number of posts received " << posts_positions.positions.size() << std::endl;
    switch(posts_positions.positions.size())
    {
        case 3:
            now_position_post_3_ = posts_positions.positions.at(2);
            flag_position_post_3_ = true;
        case 2:
            now_position_post_2_ = posts_positions.positions.at(1);
            flag_position_post_2_ = true;
        case 1:
            now_position_post_1_ = posts_positions.positions.at(0);
            flag_position_post_1_ = true;
            break;
    }
}

float Matching::calculateDistance2Points(const float &x1, const float &y1, const float &z1, 
                                         const float &x2, const float &y2, const float &z2)
{
    return sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1)));
}

float Matching::calculateDistance2Points(const geometry_msgs::Point &point_1, 
                                         const geometry_msgs::Point &point_2)
{
    return sqrt( ((point_2.x - point_1.x) * (point_2.x - point_1.x)) + 
                 ((point_2.y - point_1.y) * (point_2.y - point_1.y)) + 
                 ((point_2.z - point_1.z) * (point_2.z - point_1.z)) );
}

void Matching::drawPosts()
{
    switch(prev_positions_posts_.size())
    {
        case 3:
            marker_post_3_.pose.position = prev_positions_posts_.at(2);
            pub_marker_3_.publish(marker_post_3_);
        case 2:
            marker_post_2_.pose.position = prev_positions_posts_.at(1);
            pub_marker_2_.publish(marker_post_2_);
        case 1:
            marker_post_1_.pose.position = prev_positions_posts_.at(0);
            pub_marker_1_.publish(marker_post_1_);
            break;
    }
    switch(now_positions_posts_.size())
    {
        case 3:
            marker_post_6_.pose.position = now_positions_posts_.at(2);
            pub_marker_6_.publish(marker_post_6_);
        case 2:
            marker_post_5_.pose.position = now_positions_posts_.at(1);
            pub_marker_5_.publish(marker_post_5_);
        case 1:
            marker_post_4_.pose.position = now_positions_posts_.at(0);
            pub_marker_4_.publish(marker_post_4_);
            break;
    }
}

void Matching::savePose(const int &nPost, const geometry_msgs::Point &waypoint)
{
    switch (nPost)
    {
    case 0:
        file_post_1_ << waypoint.x << " " << waypoint.y << std::endl;
        file_post_1_time_ << ros::Time::now().toSec() - start_time_pose_ << std::endl;
        break;
    case 1:
        file_post_2_ << waypoint.x << " " << waypoint.y << std::endl;
        file_post_2_time_ << ros::Time::now().toSec() - start_time_pose_ << std::endl;
        break;
    case 2:
        file_post_3_ << waypoint.x << " " << waypoint.y << std::endl;
        file_post_3_time_ << ros::Time::now().toSec() - start_time_pose_ << std::endl;
        break;
    }
}

/** 
*   Process received post position messages
*/
void Matching::runOnce()
{
    // For starting the matching, 3 posts positions are required.
    // Until it doesn't receive those, the flag stays down (false).
    // Once 3 positions are received "simultaneosly", the flag is raised (true),
    // and will stay raised until the end of the program.
    switch (flag_matching_initialized_)
    {
    case false:
        // It initializes the matching if there are three valid posts
        if (flag_position_post_1_ && flag_position_post_2_ && flag_position_post_3_)
        {
            ROS_WARN("Init Matching");
            now_positions_posts_.push_back(now_position_post_1_);
            now_positions_posts_.push_back(now_position_post_2_);
            now_positions_posts_.push_back(now_position_post_3_);
            // Here we store the position of the post the first time that all of them are detected
            // Next code make sure the posts are ordered correctly, following next criteria.
            // post 1: port side (babor)
            // post 2: starboard side (estribor)
            // post 3: bow or furthermost post (proa)
            // The furthermost post from other ones is the 3rd post
            // Then we need to identify the other ones knowing where is the 3rd post

            // This vector stores the information about which received candidate post
            // correspond to which label(1,2,3 following explained criteria)
            // The index+1 of each element of the vector is the label to which a
            // received post is assigned, and that element is the identifier of the 
            // candidates -1 ( from 0 to 2)
            std::vector<uint8_t> matched_posts(3);

            std::vector<pair_int_float> vec_label_distance;
            float distance_post_12 = calculateDistance2Points(now_position_post_1_, 
                                                             now_position_post_2_);
            vec_label_distance.push_back(pair_int_float(12, distance_post_12));
            float distance_post_13 = calculateDistance2Points(now_position_post_1_, 
                                                             now_position_post_3_);
            vec_label_distance.push_back(pair_int_float(13, distance_post_13));
            float distance_post_23 = calculateDistance2Points(now_position_post_2_, 
                                                             now_position_post_3_);
            vec_label_distance.push_back(pair_int_float(23, distance_post_23));
            // Order distances from highest to lowest
            std::sort(vec_label_distance.begin(), vec_label_distance.end(),
                      [](const pair_int_float &l, const pair_int_float &r) {
                              if (l.second != r.second)
                                  return l.second > r.second;
                              return l.first > r.first;
                        }); 

            // Determine which distances are the two highest ones
            bool flag_distance_12_highest = false;
            bool flag_distance_13_highest = false;
            bool flag_distance_23_highest = false;
            if( (vec_label_distance[0].first == 12) || (vec_label_distance[1].first == 12) )
                flag_distance_12_highest = true;
            if( (vec_label_distance[0].first == 13) || (vec_label_distance[1].first == 13) )
                flag_distance_13_highest = true;
            if( (vec_label_distance[0].first == 23) || (vec_label_distance[1].first == 23) )
                flag_distance_23_highest = true;

            // Decide which post of the received ones is the post 3 (following our numbering scheme)
            if(flag_distance_12_highest && flag_distance_13_highest)
            {
                // Received post 1 is post 3 in our numbering scheme
                matched_posts.at(2) = 0;

            }
            else if(flag_distance_12_highest && flag_distance_23_highest)
            {
                // Received post 2 is post 3 in our numbering scheme
                matched_posts.at(2) = 1;

            }
            else if(flag_distance_13_highest && flag_distance_23_highest)
            {
                // Received post 3 is post 3 in our numbering scheme
                matched_posts.at(2) = 2;
            }

            // Now we must decide which label correspond to each remaining candidate post
            // First, we calculate the centroid of the posts
            Eigen::Vector2f centroid_posts;
            calculateCentroid3Points2D(centroid_posts, now_position_post_1_, now_position_post_2_, now_position_post_3_);
            // Then we need to transform the points coordinates to a reference system centered on posts' centroid and with 
            // x axis pointing towards 3rd post. In this way we can decide which candidate post match each label, by examining
            // its y coordinate (if it is bigger than 0 it's post 1, if its smaller then it's post 2)
            // To get the correct angle we would need to use atan2()
            float costheta, sintheta;
            Eigen::Vector2f x,y,xprima;
            x << 1, 0;
            y << 0, 1;
            xprima << now_positions_posts_[matched_posts.at(2)].x - centroid_posts[0], 
                      now_positions_posts_[matched_posts.at(2)].y - centroid_posts[1];
            costheta = xprima.dot(x)/xprima.norm();
            sintheta = xprima.dot(y)/xprima.norm();
            float theta = atan2(sintheta, costheta);

            Eigen::Matrix2f rotation_matrix;
            calculateRotationMatrix(theta, rotation_matrix);
            // Now we have the rotation angle from base reference system (velodyne) to the local reference system described above,
            // and also the translation, because its origin is the posts' centroid, that we previously calculated
            // With this, we have the transformation Tbase_local, that allows us to transform points from local reference system to
            // base reference system, but we are interested in the inverse transformation, Tlocal_base so we can obtain a point coordinates
            // in local system from base system. Plocal = Tlocal_base * Pbase
            Eigen::Affine2f Tbase_local;
            Eigen::Affine2f Tlocal_base;
            Tbase_local.translation() = centroid_posts;
            Tbase_local.linear() = rotation_matrix;
            Tlocal_base = Tbase_local.inverse();
            Eigen::Vector2f post_candidate_1, post_candidate_2, post_candidate_3;
            post_candidate_1 << now_position_post_1_.x, now_position_post_1_.y;
            post_candidate_2 << now_position_post_2_.x, now_position_post_2_.y;
            post_candidate_3 << now_position_post_3_.x, now_position_post_3_.y;

            post_candidate_1 = Tlocal_base * post_candidate_1;
            post_candidate_2 = Tlocal_base * post_candidate_2;
            post_candidate_3 = Tlocal_base * post_candidate_3;

            // Match remaining post depending on which received post is already matched
            switch(matched_posts.at(2))
            {
                // Candidate 1 already matched
                case 0:
                    // Compare y coordinates
                    if(post_candidate_2[1] < post_candidate_3[1])
                        matched_posts.at(0) = 2;
                    else
                        matched_posts.at(0) = 1;
                // Candidate 2 already matched
                case 1:                    
                    if(post_candidate_1[1] < post_candidate_3[1])
                        matched_posts.at(0) = 2;
                    else
                        matched_posts.at(0) = 0;
                // Candidate 3 already matched
                case 2:
                    if(post_candidate_1[1] < post_candidate_2[1])
                        matched_posts.at(0) = 1;
                    else
                        matched_posts.at(0) = 0;                
            }
            prev_positions_posts_.push_back(now_positions_posts_[matched_posts.at(0)]);
            prev_positions_posts_.push_back(now_positions_posts_[matched_posts.at(1)]);
            prev_positions_posts_.push_back(now_positions_posts_[matched_posts.at(2)]);

            counter_++;
            start_time_pose_ = ros::Time::now().toSec();
            now_positions_posts_.clear();
        }
        break;
    case true:
        // If there are at least two valid posts detected  
        if ( (flag_position_post_1_ && flag_position_post_2_) ||
             (flag_position_post_1_ && flag_position_post_3_) ||
             (flag_position_post_2_ && flag_position_post_3_) )
        {

            // Store the available posts positions in a vector
            if(flag_position_post_1_)
                now_positions_posts_.push_back(now_position_post_1_);

            if(flag_position_post_2_)
                now_positions_posts_.push_back(now_position_post_2_);

            if(flag_position_post_3_)
                now_positions_posts_.push_back(now_position_post_3_);

            std::vector<float> vec_check_dist(3,-1); /// Initialize with 3 elements of value -1
            std::vector<int> vec_labels(3,-1); /// This serves to store the matching between a previous post and a current post. vec_labels[i] = j
            uint32_t prev_posts_count_available = prev_positions_posts_.size();
            uint32_t now_posts_count_available = now_positions_posts_.size();
            std::vector<bool> prev_posts_indices_available(prev_posts_count_available, true);
            std::vector<bool> now_posts_indices_available(now_posts_count_available, true);
            
            // While there are previous posts and current posts (now) available for matching
            while( (prev_posts_count_available > 0) && (now_posts_count_available > 0) )
            {
                float min_distance = std::numeric_limits<float>::infinity();
                float distance;
                uint32_t prev_index_match;
                uint32_t now_index_match;
                // Find the minimum distance between all available posts
                for(uint32_t i = 0; (i < prev_positions_posts_.size()); i++)
                {
                    if(prev_posts_indices_available[i])
                    {
                        for(uint32_t j = 0; (j< now_positions_posts_.size()); j++)
                        {
                            if(now_posts_indices_available[j])
                            {
                                geometry_msgs::Point pos_aux_1 = prev_positions_posts_.at(i);
                                geometry_msgs::Point pos_aux_2 = now_positions_posts_.at(j);
                                // Calculate the distance between the waypoints of the previous state and the waypoints of the current state
                                distance = calculateDistance2Points(pos_aux_1.x, pos_aux_1.y, pos_aux_1.z,
                                                                    pos_aux_2.x, pos_aux_2.y, pos_aux_2.z);
                                if(distance < min_distance)
                                {
                                    min_distance = distance;
                                    prev_index_match = i;
                                    now_index_match = j;
                                }
                            }
                        }
                    }
                }
                //std::cout << "Found a match between previous post " << prev_index_match <<
                //" and current post " << now_index_match << std::endl;
                // Mark the matched posts as unavailable for the next iterations
                prev_posts_indices_available[prev_index_match] = false;
                now_posts_indices_available[now_index_match] = false;
                vec_labels[prev_index_match] = now_index_match;
                vec_check_dist[prev_index_match] = min_distance;
                prev_posts_count_available--;
                now_posts_count_available--;
            }

            // Save current posts positions after matching of the candidates
            for(int i = 0; i < vec_labels.size(); i++)
            {
                if (vec_labels[i] != -1)
                    prev_positions_posts_.at(i) = now_positions_posts_.at(vec_labels[i]);
            }
            //// VIRTUAL POSTS GENERATION ////
            if(flag_gen_virtual_post_)
            {
                // Eigen::Vector2f is a column vector of 2 floats
                Eigen::Vector2f v12, v13, v23, v21;
                Eigen::Vector2f v12prima, v13prima, v23prima, v21prima;
                Eigen::Vector2f x,y;
                x << 1,0; y << 0,1;

                float costheta;
                float sintheta;
                float theta;
                Eigen::Vector2f p1prima, p2prima, p3prima;
                p1prima << prev_positions_posts_.at(0).x ,prev_positions_posts_.at(0).y;
                p2prima << prev_positions_posts_.at(1).x ,prev_positions_posts_.at(1).y;
                p3prima << prev_positions_posts_.at(2).x ,prev_positions_posts_.at(2).y;
                Eigen::Matrix2f rotation_matrix;
                // Generate virtual posts positions from known ones
                // Post 1 not detected
                if(!flag_position_post_1_)
                {
                    // TODO: Adjust these values to the dimensions of the dock
                    v23 << 8.5915, 0;
                    v21 << 0.7967, 3.6132;
                    v23prima = p3prima - p2prima;
                    // Calculate angle between v23prima and v23
                    costheta = v23prima.dot(v23)/(v23prima.norm()*v23.norm());
                    sintheta = v23prima.dot(y)/(v23prima.norm());
                    theta = atan2(sintheta,costheta);
                    // Calculate v21prima from v21
                    calculateRotationMatrix(theta, rotation_matrix);
                    v21prima = rotation_matrix * v21;
                    // Get post 1 position
                    p1prima = p2prima + v21prima;
                    prev_positions_posts_.at(0).x = p1prima(0);
                    prev_positions_posts_.at(0).y = p1prima(1);
                }
                // Post 2 not detected
                else if(!flag_position_post_2_)
                {
                    // TODO: Adjust these values to the dimensions of the dock
                    v13 << 8.5915, 0;
                    v12 << 0.7967, -3.6132;
                    v13prima = p3prima - p1prima;
                    // Calculate angle between v13prima and v13
                    costheta = v13prima.dot(v13)/(v13prima.norm()*v13.norm());
                    sintheta = v13prima.dot(y)/(v13prima.norm());
                    theta = atan2(sintheta,costheta);
                    // Calculate v12prima from v12
                    calculateRotationMatrix(theta, rotation_matrix);
                    v12prima = rotation_matrix * v12;
                    // Get post 2 position
                    p2prima = p1prima + v12prima;
                    prev_positions_posts_.at(1).x = p2prima(0);
                    prev_positions_posts_.at(1).y = p2prima(1);
                }
                // Post 3 not detected
                else if(!flag_position_post_3_)
                {
                    // TODO: Adjust these values to the dimensions of the dock
                    v12 << 0,-3.7;
                    v13 << 8.39,-3.7/2;
                    v12prima = p2prima - p1prima;
                    // Calculate angle between v12prima and v12
                    costheta = v12prima.dot(v12)/(v12prima.norm()*v12.norm());
                    sintheta = v12prima.dot(x)/(v12prima.norm());
                    theta = atan2(sintheta,costheta);
                    // Calculate v13prima from v13
                    calculateRotationMatrix(theta, rotation_matrix);
                    v13prima = rotation_matrix * v13;
                    // Get post 3 position
                    p3prima = p1prima + v13prima;
                    prev_positions_posts_.at(2).x = p3prima(0);
                    prev_positions_posts_.at(2).y = p3prima(1);
                }
            }
            //////////////////////////////////////
            // Store identified posts positions
            for (int i = 0; i < prev_positions_posts_.size(); i++)
                savePose(i, prev_positions_posts_.at(i));

            // Use markers to visualize on Rviz the result
            drawPosts();
            // Publish also post positions as geometry_msgs/PointStamped
            geometry_msgs::PointStamped point_stamped;
            point_stamped.header.stamp = ros::Time::now();
            point_stamped.header.frame_id = "/velodyne";
            switch(prev_positions_posts_.size())
            {
                case 3: 
                    point_stamped.point = prev_positions_posts_.at(2);
                    pub_position_post_3_.publish(point_stamped); 
                case 2:
                    point_stamped.point = prev_positions_posts_.at(1);
                    pub_position_post_2_.publish(point_stamped);
                case 1:
                    point_stamped.point = prev_positions_posts_.at(0);
                    pub_position_post_1_.publish(point_stamped);
                default:
                    break;
            }
            // Clean vectors
            now_positions_posts_.clear();
        }
        else
        {
            // DEBUG
            //std::cout << "Not enough posts detected" << std::endl;
        }
        break;
    }
    flag_position_post_1_ = false;
    flag_position_post_2_ = false;
    flag_position_post_3_ = false;
}

void Matching::calculateRotationMatrix(const float &angle, Eigen::Matrix2f &R)
{
    R << cos(angle), -sin(angle), sin(angle), cos(angle);
}

void Matching::calculateCentroid3Points2D(Eigen::Vector2f &centroid, const geometry_msgs::Point &point_1, 
                                                                     const geometry_msgs::Point &point_2,
                                                                     const geometry_msgs::Point &point_3)
{
    centroid[0] = (point_1.x + point_2.x + point_3.x)/3;
    centroid[1] = (point_1.y + point_2.y + point_3.y)/3;
}