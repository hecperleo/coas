#include <detection/matching.h>

Matching::Matching()
{
    n = ros::NodeHandle();

    // Subscriptions
    ros::Subscriber sub_pathPoste1 = n.subscribe("/pathPoste1", 1, &Matching::cb_pathPoste1, this);
    ros::Subscriber sub_pathPoste2 = n.subscribe("/pathPoste2", 1, &Matching::cb_pathPoste2, this);
    ros::Subscriber sub_pathPoste3 = n.subscribe("/pathPoste3", 1, &Matching::cb_pathPoste3, this);
    ros::Subscriber sub_pathPoste12 = n.subscribe("/pathPoste12", 1, &Matching::cb_pathPoste12, this);
    ros::Subscriber sub_pathPoste13 = n.subscribe("/pathPoste13", 1, &Matching::cb_pathPoste13, this);
    ros::Subscriber sub_pathPoste23 = n.subscribe("/pathPoste23", 1, &Matching::cb_pathPoste23, this);

    // Publishers
    pub_marker1 = n.advertise<visualization_msgs::Marker>("/marker_post1", 1);
    pub_marker2 = n.advertise<visualization_msgs::Marker>("/marker_post2", 1);
    pub_marker3 = n.advertise<visualization_msgs::Marker>("/marker_post3", 1);
    pub_marker4 = n.advertise<visualization_msgs::Marker>("/marker_post4", 1);
    pub_marker5 = n.advertise<visualization_msgs::Marker>("/marker_post5", 1);
    pub_marker6 = n.advertise<visualization_msgs::Marker>("/marker_post6", 1);
    cont = 0;

    toDo();

    loop();
}

Matching::~Matching()
{
}

void Matching::cb_pathPoste1(const nav_msgs::Path path)
{
    nowPathPost1 = path;
}

void Matching::cb_pathPoste2(const nav_msgs::Path path)
{
    nowPathPost2 = path;
}

void Matching::cb_pathPoste3(const nav_msgs::Path path)
{
    nowPathPost3 = path;
}

void Matching::cb_pathPoste12(const nav_msgs::Path path)
{
    nowPathPost12 = path;
    toDo();
}

void Matching::cb_pathPoste13(const nav_msgs::Path path)
{
    nowPathPost13 = path;
    toDo();
}

void Matching::cb_pathPoste23(const nav_msgs::Path path)
{

    nowPathPost23 = path;
    toDo();
}

float Matching::dist2Points(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float distance;
    distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1)));
    return distance;
}

void Matching::drawPosts()
{
    marker_post1.header.frame_id = "velodyne";
    marker_post1.header.stamp = ros::Time();
    marker_post1.ns = "my_namespace";
    marker_post1.id = 0;
    marker_post1.type = visualization_msgs::Marker::CYLINDER;
    marker_post1.action = visualization_msgs::Marker::ADD;
    marker_post1.pose.position.x = prevPathPosts.poses.at(0).pose.position.x;
    marker_post1.pose.position.y = prevPathPosts.poses.at(0).pose.position.y;
    marker_post1.pose.position.z = prevPathPosts.poses.at(0).pose.position.z;
    marker_post1.pose.orientation.x = 0.0;
    marker_post1.pose.orientation.y = 0.0;
    marker_post1.pose.orientation.z = 0.0;
    marker_post1.pose.orientation.w = 1.0;
    marker_post1.scale.x = 0.6;
    marker_post1.scale.y = 0.6;
    marker_post1.scale.z = 0.6;
    marker_post1.color.a = 0.8;
    marker_post1.color.r = 1.0;
    marker_post1.color.g = 0.0;
    marker_post1.color.b = 0.0;

    marker_post2.header.frame_id = "velodyne";
    marker_post2.header.stamp = ros::Time();
    marker_post2.ns = "my_namespace";
    marker_post2.id = 0;
    marker_post2.type = visualization_msgs::Marker::CYLINDER;
    marker_post2.action = visualization_msgs::Marker::ADD;
    marker_post2.pose.position.x = prevPathPosts.poses.at(1).pose.position.x;
    marker_post2.pose.position.y = prevPathPosts.poses.at(1).pose.position.y;
    marker_post2.pose.position.z = prevPathPosts.poses.at(1).pose.position.z;
    marker_post2.pose.orientation.x = 0.0;
    marker_post2.pose.orientation.y = 0.0;
    marker_post2.pose.orientation.z = 0.0;
    marker_post2.pose.orientation.w = 1.0;
    marker_post2.scale.x = 0.6;
    marker_post2.scale.y = 0.6;
    marker_post2.scale.z = 0.6;
    marker_post2.color.a = 0.8;
    marker_post2.color.r = 0.0;
    marker_post2.color.g = 1.0;
    marker_post2.color.b = 0.0;

    marker_post3.header.frame_id = "velodyne";
    marker_post3.header.stamp = ros::Time();
    marker_post3.ns = "my_namespace";
    marker_post3.id = 0;
    marker_post3.type = visualization_msgs::Marker::CYLINDER;
    marker_post3.action = visualization_msgs::Marker::ADD;
    marker_post3.pose.position.x = prevPathPosts.poses.at(2).pose.position.x;
    marker_post3.pose.position.y = prevPathPosts.poses.at(2).pose.position.y;
    marker_post3.pose.position.z = prevPathPosts.poses.at(2).pose.position.z;
    marker_post3.pose.orientation.x = 0.0;
    marker_post3.pose.orientation.y = 0.0;
    marker_post3.pose.orientation.z = 0.0;
    marker_post3.pose.orientation.w = 1.0;
    marker_post3.scale.x = 0.6;
    marker_post3.scale.y = 0.6;
    marker_post3.scale.z = 0.6;
    marker_post3.color.a = 0.8;
    marker_post3.color.r = 0.0;
    marker_post3.color.g = 0.0;
    marker_post3.color.b = 1.0;

    marker_post4.header.frame_id = "velodyne";
    marker_post4.header.stamp = ros::Time();
    marker_post4.ns = "my_namespace";
    marker_post4.id = 0;
    marker_post4.type = visualization_msgs::Marker::CYLINDER;
    marker_post4.action = visualization_msgs::Marker::ADD;
    marker_post4.pose.position.x = nowPathPosts.poses.at(0).pose.position.x;
    marker_post4.pose.position.y = nowPathPosts.poses.at(0).pose.position.y;
    marker_post4.pose.position.z = nowPathPosts.poses.at(0).pose.position.z;
    marker_post4.pose.orientation.x = 0.0;
    marker_post4.pose.orientation.y = 0.0;
    marker_post4.pose.orientation.z = 0.0;
    marker_post4.pose.orientation.w = 1.0;
    marker_post4.scale.x = 0.3;
    marker_post4.scale.y = 0.3;
    marker_post4.scale.z = 1.0;
    marker_post4.color.a = 1.0;
    marker_post4.color.r = 0.5;
    marker_post4.color.g = 0.5;
    marker_post4.color.b = 0.0;

    marker_post5.header.frame_id = "velodyne";
    marker_post5.header.stamp = ros::Time();
    marker_post5.ns = "my_namespace";
    marker_post5.id = 0;
    marker_post5.type = visualization_msgs::Marker::CYLINDER;
    marker_post5.action = visualization_msgs::Marker::ADD;
    marker_post5.pose.position.x = nowPathPosts.poses.at(1).pose.position.x;
    marker_post5.pose.position.y = nowPathPosts.poses.at(1).pose.position.y;
    marker_post5.pose.position.z = nowPathPosts.poses.at(1).pose.position.z;
    marker_post5.pose.orientation.x = 0.0;
    marker_post5.pose.orientation.y = 0.0;
    marker_post5.pose.orientation.z = 0.0;
    marker_post5.pose.orientation.w = 1.0;
    marker_post5.scale.x = 0.3;
    marker_post5.scale.y = 0.3;
    marker_post5.scale.z = 1.0;
    marker_post5.color.a = 1.0;
    marker_post5.color.r = 0.5;
    marker_post5.color.g = 0.0;
    marker_post5.color.b = 0.5;

    marker_post6.header.frame_id = "velodyne";
    marker_post6.header.stamp = ros::Time();
    marker_post6.ns = "my_namespace";
    marker_post6.id = 0;
    marker_post6.type = visualization_msgs::Marker::CYLINDER;
    marker_post6.action = visualization_msgs::Marker::ADD;
    marker_post6.pose.position.x = nowPathPosts.poses.at(2).pose.position.x;
    marker_post6.pose.position.y = nowPathPosts.poses.at(2).pose.position.y;
    marker_post6.pose.position.z = nowPathPosts.poses.at(2).pose.position.z;
    marker_post6.pose.orientation.x = 0.0;
    marker_post6.pose.orientation.y = 0.0;
    marker_post6.pose.orientation.z = 0.0;
    marker_post6.pose.orientation.w = 1.0;
    marker_post6.scale.x = 0.3;
    marker_post6.scale.y = 0.3;
    marker_post6.scale.z = 1.0;
    marker_post6.color.a = 1.0;
    marker_post6.color.r = 0.0;
    marker_post6.color.g = 0.5;
    marker_post6.color.b = 0.5;

    pub_marker1.publish(marker_post1);
    pub_marker2.publish(marker_post2);
    pub_marker3.publish(marker_post3);
    pub_marker4.publish(marker_post4);
    pub_marker5.publish(marker_post5);
    pub_marker6.publish(marker_post6);
}

void Matching::toDo()
{
    marker_post1.action = visualization_msgs::Marker::DELETE;
    pub_marker1.publish(marker_post1);
    marker_post2.action = visualization_msgs::Marker::DELETE;
    pub_marker2.publish(marker_post2);
    marker_post3.action = visualization_msgs::Marker::DELETE;
    pub_marker3.publish(marker_post3);
    marker_post4.action = visualization_msgs::Marker::DELETE;
    pub_marker4.publish(marker_post4);
    marker_post5.action = visualization_msgs::Marker::DELETE;
    pub_marker5.publish(marker_post5);
    marker_post6.action = visualization_msgs::Marker::DELETE;
    pub_marker6.publish(marker_post6);
    // En el instante inicial se guarda la primera entrada como etiquetas válidas
    switch (cont)
    {
    case 0:
        // Se inicializa si están los tres postes validados.
        if (!nowPathPost12.poses.empty() && !nowPathPost13.poses.empty() && !nowPathPost23.poses.empty())
        {
            ROS_WARN("Init Matching");
            // Se guardan los tres waypoints relevantes del estado actual en el vector del instante anterior.
            prevPathPosts.poses.push_back(nowPathPost1.poses.at(1));
            prevPathPosts.poses.push_back(nowPathPost2.poses.at(1));
            prevPathPosts.poses.push_back(nowPathPost3.poses.at(1));
            cont++;
        }
        break;
    case 1:
        // Se ejecuta si están los tres postes validados y si los paths han cambiado.
        // if (/*prevPathPosts.poses.at(1).pose.position.x != nowPathPost1.poses.at(1).pose.position.x &&*/ (!nowPathPost12.poses.empty() && !nowPathPost13.poses.empty() || !nowPathPost23.poses.empty()))
        if ((!nowPathPost12.poses.empty() && (!nowPathPost13.poses.empty() || !nowPathPost23.poses.empty())) ||
            (!nowPathPost13.poses.empty() && (!nowPathPost12.poses.empty() || !nowPathPost23.poses.empty())) ||
            (!nowPathPost23.poses.empty() && (!nowPathPost12.poses.empty() || !nowPathPost13.poses.empty())))
        {
            // Se guardan los 6 waypoints relevantes del estado actual en un vector.
            if (!nowPathPost12.poses.empty())
            {
                nowPathPosts.poses.push_back(nowPathPost12.poses.at(0));
                nowPathPosts.poses.push_back(nowPathPost12.poses.at(1));
            }
            if (!nowPathPost13.poses.empty())
            {
                nowPathPosts.poses.push_back(nowPathPost13.poses.at(0));
                nowPathPosts.poses.push_back(nowPathPost13.poses.at(1));
            }
            if (!nowPathPost23.poses.empty())
            {
                nowPathPosts.poses.push_back(nowPathPost23.poses.at(0));
                nowPathPosts.poses.push_back(nowPathPost23.poses.at(1));
            }
            std::vector<int> vec_labels;
            std::vector<float> vec_check_dist;
            // Se compara cada waypoint relevante del estado anterior con los waypoints del estado actual.
            for (int i = 0; i < prevPathPosts.poses.size(); i++)
            {
                for (int j = 0; j < nowPathPosts.poses.size(); j++)
                {
                    // Se calcula la distancia que hay entre los waypoints del instante actual y el instante anterior.
                    vec_distAndLabel[j] = dist2Points(prevPathPosts.poses.at(i).pose.position.x, prevPathPosts.poses.at(i).pose.position.y, prevPathPosts.poses.at(i).pose.position.z,
                                                      nowPathPosts.poses.at(j).pose.position.x, nowPathPosts.poses.at(j).pose.position.y, nowPathPosts.poses.at(j).pose.position.z);

                    vec_dist.push_back(dist2Points(prevPathPosts.poses.at(i).pose.position.x, prevPathPosts.poses.at(i).pose.position.y, prevPathPosts.poses.at(i).pose.position.z,
                                                   nowPathPosts.poses.at(j).pose.position.x, nowPathPosts.poses.at(j).pose.position.y, nowPathPosts.poses.at(j).pose.position.z));
                }

                // Se ordenan todas las distancias resultantes de menor a mayor.
                std::vector<pair> vec;
                std::copy(vec_distAndLabel.begin(), vec_distAndLabel.end(), std::back_inserter<std::vector<pair>>(vec));
                std::sort(vec.begin(), vec.end(),
                          [](const pair &l, const pair &r) {
                              if (l.second != r.second)
                                  return l.second < r.second;
                              return l.first < r.first;
                          });

                vec_labels.push_back(vec[0].first);
                vec_check_dist.push_back(vec[0].second);
                vec_dist.clear();
            }
            // Se limpia el vector de waypoints del instante anterios y se le coloca el frame_id correspondiente.
            prevPathPosts.poses.clear();
            prevPathPosts.header.frame_id = "velodyne";
            // [PARA .BAG CON LOOP] Se revisan todas las distancias y si hay mucha diferencia, se asume que el estado actual es el bueno y, por tanto, se tiene que volver a inicializar.
            // for (int i = 0; i < vec_check_dist.size(); i++)
            // {
            //     if (vec_check_dist[i] > 2.0)
            //     {
            //         cont = 0;
            //         ROS_WARN("Reset Matching");
            //         break;
            //     }
            // }
            // // [PARA .BAG CON LOOP] Se sale de el estado actual y reinicia el algoritmo.
            // if (cont == 0)
            // {
            //     break;
            // }
            // Se guarda como instante anterior, los postes con el identificador correctamente colocado.
            for (int i = 0; i < vec_labels.size(); i++)
            {
                prevPathPosts.poses.push_back(nowPathPosts.poses.at(vec_labels[i]));
            }
            // Se utilizan Markers para visualizar en Rviz el resultado.
            drawPosts();
            // Limpia vectores.
            vec_labels.clear();
            vec_check_dist.clear();
            vec_distAndLabel.clear();
            nowPathPosts.poses.clear();
            nowPathPost23.poses.clear();
        }
        break;
    }
}

void Matching::loop()
{
    while (ros::ok())
    {
        sleep(0.1);
        ros::spinOnce();
    }
}