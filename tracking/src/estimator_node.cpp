//------------------------------------------------------------------------------
// GRVC 
// Author Jesus Capitan <jcapitan@us.es>
//------------------------------------------------------------------------------
//
// Copyright (c) 2016 GRVC University of Seville
//
//------------------------------------------------------------------------------

#include <ros/ros.h>

#include <tracking/centralized_estimator.h>
#include <tracking/candidate.h>
#include <tracking/CandidateMsg.h>

#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>

#include <string>
#include <vector>

using namespace std;

/** This class implements the Estimator node with a centralized estimator for target tracking.
*/
class EstimatorNode 
{

public:
	EstimatorNode();
	~EstimatorNode();

protected:

	/// Callbacks
	void candidatesReceived(const tracking::CandidateMsg::ConstPtr& candidate_msg);
	
	void publishBelief();
	void eigendec(double c11, double c22, double c12, vector<double> &D, vector<double> &E);
	
	/// Node handlers
	ros::NodeHandle* nh_; 
	ros::NodeHandle* pnh_;

	/// Subscribers
	ros::Subscriber candidate_sub_;

	/// Publishers
	ros::Publisher belief_pub_;

	/// Candidates queues
	vector<Candidate *> candidates_;

	/// Estimator frequency
	double estimator_rate_;

	/// Maximum delay allowed for candidates (seconds)
	double delay_max_;

	/// Reference frame in which all messages are received and published
	string reference_frame_;

	/// Centralized filter for target estimation 
	CentralizedEstimator* estimator_;
	
};

/** \brief Constructor
*/
EstimatorNode::EstimatorNode()
{
	nh_ = new ros::NodeHandle();
	pnh_ = new ros::NodeHandle("~");

	double lost_time_th, association_th;
	int min_update_count;

	// Read parameters
	pnh_->param<double>("estimator_rate", estimator_rate_, 5.0); 
	pnh_->param<double>("lost_time_th", lost_time_th, 20.0);
	pnh_->param<int>("min_update_count", min_update_count, 0); 
	pnh_->param<double>("association_th", association_th, 6.0);
	pnh_->param<double>("delay_max", delay_max_, 2.0);
	pnh_->param<string>("reference_frame", reference_frame_,"/map");
	
	// Estimator and allocator
	estimator_ = new CentralizedEstimator(association_th, lost_time_th, min_update_count);
	
	// Subscriptions/publications
	candidate_sub_ = nh_->subscribe<tracking::CandidateMsg>("candidates", 1, &EstimatorNode::candidatesReceived, this);
	belief_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("targets_belief", 1);

	// Main loop

	ros::Rate r(estimator_rate_);
	ros::Time prev_time, time_now;

	int count = 0;

	while(nh_->ok())
	{
		// Execute all pending callbacks until this moment
		ros::spinOnce();

		// Predict estimator
		time_now = ros::Time::now();
		double elapsed_time = (time_now - prev_time).toSec();
		prev_time = time_now;

		if(elapsed_time)
			estimator_->predict(elapsed_time);

		// Update
		estimator_->update(candidates_); 
		
		// Remove candidates
		for(int i = 0; i < candidates_.size(); i++)
			delete candidates_[i];
	
		candidates_.clear();

		estimator_->removeLostTargets();
		
		publishBelief();

		if(count == 5)
		{
			estimator_->printTargetsInfo();
			count = 0;
		}
		else
			count++;

		r.sleep();
	}
}

/** \brief Destructor
*/
EstimatorNode::~EstimatorNode()
{
	delete nh_;
	delete pnh_;
	delete estimator_;

	for(int i = 0; i < candidates_.size(); i++)
		delete candidates_[i];
	
	candidates_.clear();
}

/** \brief Callback to receive observations from vision module
*/
void EstimatorNode::candidatesReceived(const tracking::CandidateMsg::ConstPtr& candidate_msg)
{
	double delay = (ros::Time::now() - candidate_msg->header.stamp).toSec();
	
	if(delay < delay_max_)
	{		
		Candidate* cand_p = new Candidate;
		
		cand_p->size = candidate_msg->size;
		cand_p->location(0) = candidate_msg->location.x;
		cand_p->location(1) = candidate_msg->location.y;
		cand_p->speed(0) = candidate_msg->speed.x;
		cand_p->speed(1) = candidate_msg->speed.y;
		
		cand_p->location_covariance.setZero(2,2);
		cand_p->speed_covariance.setZero(2,2);
		
		for(int i = 0; i < 2; i++)
		{
			for(int j = 0; j < 2; j++)
			{
				cand_p->location_covariance(i,j) = candidate_msg->location_covariance[i*2+j];
				cand_p->speed_covariance(i,j) = candidate_msg->speed_covariance[i*2+j];
			}	
		}
	
		candidates_.push_back(cand_p);
	}
	else
		ROS_WARN("Received candidate with long delay");
}

/** \brief Publish markers to represent targets beliefs
*/
void EstimatorNode::publishBelief()
{
	visualization_msgs::MarkerArray marker_array;
	ros::Time curr_time = ros::Time::now();

	vector<double> w(2);
	vector<double> v(4);
	vector<vector<double> > covariances;
	double a, b, yaw, x, y, vx, vy;
	TargetStatus target_status;
	ObjSize target_size;

	// Get ids to plot active targets
	vector<int> active_targets = estimator_->getActiveTargets();

	for(int i = 0; i < active_targets.size(); i++)
	{
		if(estimator_->getTargetInfo(active_targets[i], x, y, covariances, vx, vy))
		{
			estimator_->getTargetInfo(active_targets[i], x, y, target_status, target_size);

			// Compute SVD of cholesky. The singular values are the square roots of the eigenvalues of
			// the covariance matrix 
			eigendec(4*covariances[0][0], 4*covariances[1][1], 4*covariances[0][1], w, v);

			a = sqrt(fabs(w[0]));
			b = sqrt(fabs(w[1]));

			yaw = atan2(v[1],v[0]);
			
			// Fill in marker
			visualization_msgs::Marker marker;

			// Set the frame ID and timestamp
			marker.header.frame_id = reference_frame_;    
			marker.header.stamp = curr_time;

			// Set the namespace and id for this marker.  This serves to create a unique ID    
			// Any marker sent with the same namespace and id will overwrite the old one    
			marker.ns = "cov_ellipse";    
			marker.id = i;
		
			// Set the marker type    
			marker.type = visualization_msgs::Marker::SPHERE;

			// Set the marker action.  Options are ADD and DELETE    
			marker.action = visualization_msgs::Marker::ADD;
			
			// Set the scale of the marker -- 1x1x1 here means 1m on a side

			marker.scale.x = a;
			marker.scale.y = b;    
			marker.scale.z = 0.1;

			marker.lifetime = ros::Duration(1.0);

			// Set the central pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header    
			marker.pose.position.x = x;
			marker.pose.position.y = y;
			marker.pose.position.z = 0;
			
			tf2::Quaternion q;
			q.setRPY(0.0,0.0,yaw);
			marker.pose.orientation = tf2::toMsg(q);

			marker_array.markers.push_back(marker);

			// Plot velocity
			marker.ns = "velocity";
			marker.type = visualization_msgs::Marker::ARROW;    
			marker.scale.x = sqrt(vx*vx+vy*vy);
			marker.scale.y = 0.1;    
			marker.scale.z = 0.1;
			if(marker.scale.x != 0.0)
			{
				tf2::Quaternion q;
				q.setRPY(0.0,0.0,atan2(vy,vx));
				marker.pose.orientation = tf2::toMsg(q);
			}
			
			marker_array.markers.push_back(marker);

			// Plot target size
			marker.ns = "target_size";
			marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker.text = to_string(target_size);
			marker.pose.position.x -= 1.0;
			marker.pose.position.y -= 1.0;    
			marker.scale.z = 1.0;
			
			marker_array.markers.push_back(marker);

			// Plot target ID
			marker.ns = "target_id";
			marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker.text = to_string(active_targets[i]);
			marker.pose.position.x += 1.0;
			marker.pose.position.y += 1.0;    
			marker.scale.z = 1.0;
			
			marker_array.markers.push_back(marker);
		}
		else
			ROS_ERROR("Target ID not found");
	}
	// Publish the marker    
	belief_pub_.publish(marker_array);
}

/**
 Closed form eigenvalue decomposition
 
 C  Positive definite input matrix of form
        I[0] I[1]
        I[2] I[3]   where I[1]=I[2]
 D  Output eigenvalues
 E  Eigenvector matrix of form
        E[0] E[2]
        E[1] E[3]
*/
void EstimatorNode::eigendec(double c11, double c22, double c12, vector<double> &D, vector<double> &E)
{ 
	double a,b,enorm;

	// Find eigenvalues 
	a = c11+c22;                         // trace 
 	b = sqrt((c11-c22)*(c11-c22)+4*c12*c12);
 	D[0] = (a+b)/2;
 	D[1] = (a-b)/2;

	// Find eigenvector 1 
	E[0] = c22+c12-D[0];
	E[1] = D[0]-c11-c12;
	enorm = sqrt(E[0]*E[0]+E[1]*E[1]);

	if(enorm > 0.0) {
		E[0] = E[0]/enorm;
		E[1] = E[1]/enorm;
	}

	// Find eigenvector 2 
	E[2] = c22+c12-D[1];
	E[3] = D[1]-c11-c12;
	enorm = sqrt(E[2]*E[2]+E[3]*E[3]);

	if(enorm > 0.0) {
		E[2] = E[2]/enorm;
		E[3] = E[3]/enorm;
	}
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "estimator_node");
  
  EstimatorNode estimator; 
  
  return (0);
}
