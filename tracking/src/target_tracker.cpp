//------------------------------------------------------------------------------
// GRVC 
// Author Jesus Capitan <jcapitan@us.es>
//------------------------------------------------------------------------------
//
// Copyright (c) 2016 GRVC University of Seville
//
//------------------------------------------------------------------------------

#include <tracking/target_tracker.h>
#include <ros/duration.h>
#include <iostream>

#define VEL_NOISE_VAR 0.2 
#define SIZE_DETECTOR_PD 0.9
#define MIN_SIZE_DISTANCE 0.15

using namespace std;

/** Constructor
\param id Identifier
*/
TargetTracker::TargetTracker(int id)
{
	//TODO: This member variable can be used to store the MMSI when the obstacle
	// is a ship and its MMSI its known
	id_ = id;

	is_static_ = false;
	status_ = ACTIVE;

	fact_bel_.resize(1);
	fact_bel_[OBJSIZE].resize(N_SIZES);

	pose_ = Eigen::MatrixXd::Zero(4,1);
	pose_cov_ = Eigen::MatrixXd::Identity(4,4);
}

/// Destructor
TargetTracker::~TargetTracker()
{
}

/**
\brief Initialize the filter. 
\param z Initial observation
*/
void TargetTracker::initialize(Candidate* z)
{
	// Setup state vector
	pose_.setZero(4, 1);
	pose_(0,0) = z->location(0);
	pose_(1,0) = z->location(1);
	pose_(2,0) = 0.0;
	pose_(3,0) = 0.0;
		
	// Setup cov matrix
	pose_cov_.setIdentity(4, 4);
	pose_cov_(0,0) = 2*z->location_covariance(0,0);
	pose_cov_(0,1) = 2*z->location_covariance(0,1);
	pose_cov_(1,0) = 2*z->location_covariance(1,0);
	pose_cov_(1,1) = 2*z->location_covariance(1,1);

	pose_cov_(2,2) = VEL_NOISE_VAR;
	pose_cov_(3,3) = VEL_NOISE_VAR;

	// Init and update factored belief 
	for(int fact = 0; fact < fact_bel_.size(); fact++)
	{
		switch(fact)
		{
			case OBJSIZE:

			double prob_z, total_prob = 0.0;

			for(int i = 0; i < N_SIZES; i++)
			{
				if(z->size == i)
					prob_z = SIZE_DETECTOR_PD;
				else
					prob_z = (1.0 - SIZE_DETECTOR_PD)/(N_SIZES-1);

				fact_bel_[OBJSIZE][i] = (1.0/(N_SIZES))*prob_z;
				total_prob += fact_bel_[OBJSIZE][i];
			}
			
			// Normalize
			for(int i = 0; i < N_SIZES; i++)
			{
				fact_bel_[OBJSIZE][i] /= total_prob;
			}	

			break;			
		}
	}

	is_static_ = false;

	// Update timer
	update_timer_.reset();
	update_count_ = 0;
}

/**
\brief Predict the filter.
\param dt Length in seconds of the prediction step. 
*/
void TargetTracker::predict(double dt)
{
	// static factors do not vary. Position depending on whether it is dynamic or not.

	if(!is_static_)
	{
		// State vector prediction
		pose_(0,0) += pose_(2,0)*dt;
		pose_(1,0) += pose_(3,0)*dt;
		
		// Convariance matrix prediction
		Eigen::Matrix<double, 4, 4> F;
		F.setIdentity(4, 4);
		F(0,2) = dt;
		F(1,3) = dt;
		Eigen::Matrix<double, 4, 4> Q;
		Q.setZero(4, 4);
		Q(2,2) = VEL_NOISE_VAR*dt*dt;
		Q(3,3) = VEL_NOISE_VAR*dt*dt;
		pose_cov_ = F*pose_cov_*F.transpose() + Q;
	}
}

/**
\brief Update the filter.
\param z Observation to update. 
\return True if everything was fine
*/
bool TargetTracker::update(Candidate* z)
{
	// Update factored belief 
	for(int fact = 0; fact < fact_bel_.size(); fact++)
	{
		switch(fact)
		{
			case OBJSIZE:

			double prob_z, total_prob = 0.0;

			for(int i = 0; i < N_SIZES; i++)
			{
				if(z->size == i)
					prob_z = SIZE_DETECTOR_PD;
				else
					prob_z = (1.0 - SIZE_DETECTOR_PD)/(N_SIZES-1);

				fact_bel_[OBJSIZE][i] *= prob_z;
				total_prob += fact_bel_[OBJSIZE][i];
			}
			
			// Normalize
			for(int i = 0; i < N_SIZES; i++)
			{
				fact_bel_[OBJSIZE][i] /= total_prob;
			}	

			break;			
		}
	}
	
	// Compute update jacobian
	Eigen::Matrix<double, 2, 4> H;
	H.setZero(2, 4);
	H(0,0) = 1.0;
	H(1,1) = 1.0;
		
	// Compute update noise matrix
	Eigen::Matrix<double, 2, 2> R;
	R(0,0) = z->location_covariance(0,0);
	R(0,1) = z->location_covariance(0,1);
	R(1,0) = z->location_covariance(1,0);
	R(1,1) = z->location_covariance(1,1);
		
	// Calculate innovation matrix
	Eigen::Matrix<double, 2, 2> S;
	S = H*pose_cov_*H.transpose() + R;
		
	// Calculate kalman gain
	Eigen::Matrix<double, 4, 2> K;
	K = pose_cov_*H.transpose()*S.inverse();
		
	// Calculate innovation vector
	Eigen::Matrix<double, 2, 1> y;
	y = H*pose_;
	y(0,0) = z->location(0) - y(0,0);
	y(1,0) = z->location(1) - y(1,0);
		
	// Calculate new state vector
	pose_ = pose_ + K*y;
	
	//TODO: Speed data should be updated whenever is available

	// Calculate new cov matrix
	Eigen::Matrix<double, 4, 4> I;
	I.setIdentity(4, 4);
	pose_cov_ = (I - K*H)*pose_cov_;

	// Update timer
	update_timer_.reset();
	update_count_++;
}
    
/**
Compute the likelihood of an observation with current belief. Based on Mahalanobis distance. 
\param z Observation. 
\return Likelihood measurement
*/
double TargetTracker::getMahaDistance(Candidate* z)
{
	double distance;

	// Compute update jacobian
	Eigen::Matrix<double, 2, 4> H;
	H.setZero(2, 4);
	H(0,0) = 1.0;
	H(1,1) = 1.0;
		
	// Compute update noise matrix
	Eigen::Matrix<double, 2, 2> R;
	R(0,0) = z->location_covariance(0,0);
	R(0,1) = z->location_covariance(0,1);
	R(1,0) = z->location_covariance(1,0);
	R(1,1) = z->location_covariance(1,1);
		
	// Calculate innovation matrix
	Eigen::Matrix<double, 2, 2> S;
	S = H*pose_cov_*H.transpose() + R;
			
	// Calculate innovation vector
	Eigen::Matrix<double, 2, 1> y;
	y = H*pose_;
	y(0,0) = z->location(0) - y(0,0);
	y(1,0) = z->location(1) - y(1,0);

	// This is a squared distance
	distance = y.transpose()*S.inverse()*y;
	// Get non squared distance
	distance = sqrt(distance);
	double prob_z, prob_size = 0.0;

	/*
	for(int i = 0; i < N_SIZES; i++)
	{
		if(z->size == i)
			prob_z = SIZE_DETECTOR_PD;
		else
			prob_z = (1.0 - SIZE_DETECTOR_PD)/(N_SIZES-1);

		prob_size += fact_bel_[OBJSIZE][i]*prob_z;
	}

	//TODO: Adjust the threshold to this particular use case
	if(prob_size < MIN_SIZE_DISTANCE)
		distance = -1;
	*/
	return distance;
}

/**
Compute the euclidean distance of an observation with current belief.
\param z Observation. 
\return Euclidean distance
*/
double TargetTracker::getDistance(Candidate* z)
{
	double dx, dy;
	dx = pose_(0,0) - z->location(0);
	dy = pose_(1,0) - z->location(1);

	return sqrt(dx*dx + dy*dy);
}

/**
Return the time since the last observation update. 
\return Update time
*/
ros::Duration TargetTracker::lastUpdateTime()
{
	return update_timer_.elapsed();
}

/**
Return the counter of updates. 
\return Update counter
*/
int TargetTracker::getUpdateCount()
{
	return update_count_;
}
    
/** \brief Return pose information from the target
\param x Position of the target
\param y Position of the target
*/
void TargetTracker::getPose(double &x, double &y)
{
	x = pose_(0,0);
	y = pose_(1,0);
}

/** \brief Return velocity information from the target
\param vx Velocity of the target
\param vy Velocity of the target
*/
void TargetTracker::getVelocity(double &vx, double &vy)
{
	vx = pose_(2,0);
	vy = pose_(3,0);
}

/** \brief Return covariance matrix from the target position
\return Covariance matrix
*/
vector<vector<double> > TargetTracker::getCov()
{
	vector<vector<double> > covariance;
	covariance.resize(2);
	covariance[0].resize(2);
	covariance[1].resize(2);

	covariance[0][0] = pose_cov_(0,0);
	covariance[0][1] = pose_cov_(0,1);
	covariance[1][0] = pose_cov_(1,0);
	covariance[1][1] = pose_cov_(1,1);

	return covariance;
}

/** \brief Return number of discrete factors
\return Number of discrete factors
*/
int TargetTracker::getNumFactors()
{
	return fact_bel_.size();
}

/** \brief Return probabilities for a given factor
\param factor Factor to return
\return Probabilities for a given factor
*/
vector<double> TargetTracker::getFactorProbs(int factor)
{
	return fact_bel_[factor];
}

/** \brief Return likeliest target status
\return Target status  
*/
TargetStatus TargetTracker::getStatus()
{
	return status_;
}

/** \brief Set a new target status
\param Target status  
*/
void TargetTracker::setStatus(TargetStatus status)
{
	status_ = status;
}

/** \brief Return target identifier
\return Target identifier  
*/
int TargetTracker::getId()
{
	return id_;
}

/** \brief Return whether target is static or not
\return True if it is static
*/
bool TargetTracker::isStatic()
{
	return is_static_;
}

/** \brief Return the likeliest size for the target
\return A size
*/
ObjSize TargetTracker::getObjSize()
{
	double max_prob = -1.0;
	ObjSize size, size_max;

	// With this, if some sizes have the same probability and it is the highest one,
	// it considers the object has the highest size from the ones that shares the same
	// probability
	for(int i = 0; i < N_SIZES; i++)
	{
		if(fact_bel_[OBJSIZE][i] > max_prob)
		{
			max_prob = fact_bel_[OBJSIZE][i];
			size_max = (ObjSize)i;
		}
	}

	if(max_prob > 1.0 - max_prob)
		size = size_max;
	else
		size = UNKNOWN;

	return size;
}
