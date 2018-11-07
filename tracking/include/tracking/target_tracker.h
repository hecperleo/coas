//------------------------------------------------------------------------------
// GRVC 
// Author Jesus Capitan <jcapitan@us.es>
//------------------------------------------------------------------------------
//
// Copyright (c) 2016 GRVC University of Seville
//
//------------------------------------------------------------------------------

#ifndef TARGET_TRACKER_H_
#define TARGET_TRACKER_H_

#include <tracking/candidate.h>
#include <tracking/timer.hpp>

#include <vector>
#include <Eigen/Eigen>


enum TargetStatus {ACTIVE, LOST, N_STATUS};
enum ObjSize {UNKNOWN = -1, SMALL = 0, MEDIUM, LARGE, N_SIZES};
enum Factor {OBJSIZE};
    
/** \brief This class implements a stochastic filter for an object. 

This class implements a stochastic filter for an object. The filter estimates some continuous features 
(e.g., the position and velocity) and some discrete features (e.g., size). The object may be static or 
moving. 

*/

class TargetTracker 
{
public:
	TargetTracker(int id);
	~TargetTracker();

	void initialize(Candidate* z);
	void predict(double dt);
	bool update(Candidate* z);
	double getMahaDistance(Candidate* z);
	double getDistance(Candidate* z);
	double lastUpdateTime();
	int getUpdateCount();
	void getPose(double &x, double &y);
	void getVelocity(double &vx, double &vy);
	std::vector<std::vector<double> > getCov();
	int getNumFactors();
	std::vector<double> getFactorProbs(int factor);

	TargetStatus getStatus();
	void setStatus(TargetStatus status);
	int getId();
	bool isStatic();
	ObjSize getObjSize();
	
protected:
	Timer update_timer_;			/// Timer for last update
	int update_count_;				/// Counter with the number of updates
	int id_;						/// Target identifier
	bool is_static_;				/// It indicates whether the target is static/dynamic
	TargetStatus status_;			/// Current status

	/// Factored discrete belief: COLOR	
	std::vector<std::vector<double> > fact_bel_;	
	
	/// State vector: [x (m), y (m), vx (m/s), vy (m/s)]
	Eigen::MatrixXd pose_;
	Eigen::MatrixXd pose_cov_;

};

#endif
