//------------------------------------------------------------------------------
// GRVC
// Author Jesus Capitan <jcapitan@us.es>
//------------------------------------------------------------------------------
// Copyright (c) 2016 GRVC University of Seville
//
//------------------------------------------------------------------------------

#ifndef CENTRALIZED_ESTIMATOR_H_
#define CENTRALIZED_ESTIMATOR_H_

#include <map>
#include <vector>

#include <tracking/target_tracker.h>


/** \brief This class implements a centralized filter to detects obstacles.

This class implements a centralized filter for all the objects around. A table with all objects is maintained, and a filter for each object, static or moving. 

*/
class CentralizedEstimator
{
public:

	CentralizedEstimator(double lkhd_th, double lost_th, int min_update_count, bool use_mahalanobis_distance);
	~CentralizedEstimator();

	void predict(double dt);
	bool update(std::vector<Candidate*> cand_list);

	int getNumTargets();
	std::vector<int> getActiveTargets();
	bool getTargetInfo(int target_id, double &x, double &y, TargetStatus &type, ObjSize &size);
	bool getTargetInfo(int target_id, double &x, double &y, std::vector<std::vector<double> > &covariances);
	bool getTargetInfo(int target_id, double &x, double &y, std::vector<std::vector<double> > &covariances, double &vx, double &vy);
	bool setTargetStatus(int target_id, TargetStatus status);
	void removeLostTargets();
	void printTargetsInfo();

protected:

	std::map<int, TargetTracker *> targets_;	/// Map with targets
	double likelihood_th_;						/// Minimum likelihood threshold for data association
	double lost_th_;							/// Maximum time threshold to lose target
	int min_update_count_;						/// Minimum update counter to consider target consistent
	int track_id_count_;						/// Counter of tracks identifiers
	bool f_use_maha_distance_;					/// Flag: When true use Mahalanobis distance, when false use euclidean distance to match candidates and targets
};

#endif
