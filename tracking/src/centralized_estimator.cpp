//------------------------------------------------------------------------------
// GRVC 
// Author Jesus Capitan <jcapitan@us.es>
//------------------------------------------------------------------------------
//
// Copyright (c) 2016 GRVC University of Seville
//
//------------------------------------------------------------------------------

#include <tracking/centralized_estimator.h>
#include <iostream>

// #define DEBUG_MODE

using namespace std;

/** Constructor
\param likelihood_th Likelihood threshold to associate observations
\param lost_th Time threshold to consider target lost
\param min_update_count Minimum number of updates to consider a target consistent 
*/
CentralizedEstimator::CentralizedEstimator(double lkhd_th, double lost_th, int min_update_count, bool use_mahalanobis_distance)
{
	likelihood_th_ = lkhd_th;
	lost_th_ = lost_th;
	min_update_count_ = min_update_count;
	track_id_count_ = 0;
	f_use_maha_distance_ = use_mahalanobis_distance;
}

/// Destructor
CentralizedEstimator::~CentralizedEstimator()
{
	// Free up memory
	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		delete(it->second);
	}
}

/** Prediction step for all TargetStatus
\param dt Length in seconds of the prediction step.
*/
void CentralizedEstimator::predict(double dt)
{
	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		(it->second)->predict(dt);
	}
}

/**
\brief Update step for a target
\param cand_list List with observations to update
\return True if everything was fine
*/
bool CentralizedEstimator::update(vector<Candidate*> cand_list)
{
	vector<vector<double> > distances;
	vector<int> valid_targets;
	vector<int> valid_candidates;
	int n_valid_targets = 0, n_valid_candidates = 0;

	// Compute distances for each association. And count valid targets
	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		bool target_valid = true;

		if((it->second)->getStatus() == ACTIVE)
		{
			valid_targets.push_back((it->second)->getId());
				n_valid_targets++;
		}
		else
		{
			valid_targets.push_back(-1);
			target_valid = false;
		}

		vector<double> t_distances;
		double likelihood;

		for(int i = 0; i < cand_list.size(); i++)
		{
			if(target_valid)
			{		
				if(f_use_maha_distance_)
					likelihood = (it->second)->getMahaDistance(cand_list[i]);
				else
					likelihood = (it->second)->getDistance(cand_list[i]);
				t_distances.push_back(likelihood);
			}
			else
				t_distances.push_back(-1.0);
		}

		distances.push_back(t_distances);
	}

	// All candidates valid initially
	for(int i = 0; i < cand_list.size(); i++)
		valid_candidates.push_back(1);

	n_valid_candidates = cand_list.size();

	#ifdef DEBUG_MODE
	cout << "Candidates " << endl;
	cout << "Distances: " << endl;
	for(int i = 0; i < distances.size(); i++)
	{
		for(int j = 0; j < distances[i].size(); j++)
			cout << distances[i][j] << " ";
		cout << endl;
	}
	cout << endl;	
	
	#endif

	// Look for best pairs until running out of candidates or targets
	while(n_valid_targets != 0 && n_valid_candidates != 0 )
	{
		double min_dist = -1.0;
		pair<int, int> best_pair;

		for(int t_id = 0; t_id < distances.size(); t_id++)
		{
			if(valid_targets[t_id] != -1)
			{
				for(int c_id = 0; c_id < distances[t_id].size(); c_id++)
				{
					if(valid_candidates[c_id] != -1 && distances[t_id][c_id]!= -1 && (min_dist == -1.0 || distances[t_id][c_id] < min_dist))
					{
						min_dist = distances[t_id][c_id];
						best_pair.first = t_id;
						best_pair.second = c_id;
					}		
				}
			}
		}

		// If there is no good data association, create new target
		if(min_dist != -1 && min_dist <= likelihood_th_)
		{
			#ifdef DEBUG_MODE
			cout << "Candidate " << cand_list[best_pair.second]->location(0) << "," << cand_list[best_pair.second]->location(1) << ". Associated to target " << valid_targets[best_pair.first] << ", with distance " << min_dist << endl;
			#endif

			targets_[valid_targets[best_pair.first]]->update(cand_list[best_pair.second]);
			// Podríamos hacer lo siguiente cuando encontramos una correspondencia
			// valid_targets[best_pair.first] = -1
			// Pero puede ser que haya varios candidatos que se correspondan con el mismo target
		}
		else
		{
			#ifdef DEBUG_MODE
			cout << "Candidate " << cand_list[best_pair.second]->location(0) << "," << cand_list[best_pair.second]->location(1) << ". New target " << track_id_count_ << ", with distance " << min_dist << endl;
			#endif
			int new_target_id = track_id_count_++;
			targets_[new_target_id] = new TargetTracker(new_target_id);
			targets_[new_target_id]->initialize(cand_list[best_pair.second]);

			// Include new target's distances
			valid_targets.push_back(new_target_id);
			n_valid_targets++;

			vector<double> t_distances;
			double likelihood;

			for(int i = 0; i < cand_list.size(); i++)
			{
				if(valid_candidates[i] != -1)
				{		
					likelihood = targets_[new_target_id]->getMahaDistance(cand_list[i]);
					t_distances.push_back(likelihood);
				}
				else
					t_distances.push_back(-1.0);
			}

			distances.push_back(t_distances);			
		}

		valid_candidates[best_pair.second] = -1;
		n_valid_candidates--;
	}

	// Create new targets with remaining candidates
	if(n_valid_candidates)
	{
		for(int i = 0; i < cand_list.size(); i++)
		{
			if(valid_candidates[i])
			{
				#ifdef DEBUG_MODE
				cout << "Candidate " << cand_list[i]->location(0) << "," << cand_list[i]->location(1) << ". New target " << track_id_count_ << endl;
				#endif

				targets_[track_id_count_] = new TargetTracker(track_id_count_);
				targets_[track_id_count_++]->initialize(cand_list[i]);
			}
		}
	}
}

/// Return number of targets
int CentralizedEstimator::getNumTargets()
{
	return targets_.size();
}

/// Return Identifiers of active targets not caught, lost or deployed
vector<int> CentralizedEstimator::getActiveTargets()
{
	vector<int> targets_ids;

	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		if( (it->second)->getStatus() == ACTIVE )
		{
			targets_ids.push_back((it->second)->getId());
		}
	}

	return targets_ids;
}

/** \brief Return information from a target
\param target_id Identifier of the target
\param x Position of the target
\param y Position of the target
\param Status Status of the target 
\param size Size of the target
\return True if the target was found 
*/
bool CentralizedEstimator::getTargetInfo(int target_id, double &x, double &y, TargetStatus &status, ObjSize &size)
{
	bool found = false;

	auto it = targets_.find(target_id);
	if(it != targets_.end())
	{
		found = true;
		targets_[target_id]->getPose(x, y);
		status = targets_[target_id]->getStatus();
		size = targets_[target_id]->getObjSize();
	}
	
	return found;
}

/** \brief Return position information from a target
\param target_id Identifier of the target
\param x Position of the target
\param y Position of the target
\param covariance Covariance matrix for position
\return True if the target was found 
*/
bool CentralizedEstimator::getTargetInfo(int target_id, double &x, double &y, vector<vector<double> > &covariances)
{
	bool found = false;

	auto it = targets_.find(target_id);
	if(it != targets_.end())
	{
		found = true;
		targets_[target_id]->getPose(x, y);
		covariances = targets_[target_id]->getCov();
	}
	
	return found;
}

/** \brief Return position and velocity information from a target
\param target_id Identifier of the target
\param x Position of the target
\param y Position of the target
\param covariance Covariance matrix for position
\param vx Velocity of the target
\param vy Velocity of the target
\return True if the target was found 
*/
bool CentralizedEstimator::getTargetInfo(int target_id, double &x, double &y, vector<vector<double> > &covariances, double &vx, double &vy)
{
	bool found = false;

	auto it = targets_.find(target_id);
	if(it != targets_.end())
	{
		found = true;
		targets_[target_id]->getPose(x, y);
		covariances = targets_[target_id]->getCov();
		targets_[target_id]->getVelocity(vx, vy);
	}
	
	return found;
}

/** \brief Set the status of a target
\param target_id Identifier of the target
\param Status Status of the target 
\return True if the target was found 
*/
bool CentralizedEstimator::setTargetStatus(int target_id, TargetStatus status)
{
	bool found = false;

	auto it = targets_.find(target_id);
	if(it != targets_.end())
	{
		found = true;
		targets_[target_id]->setStatus(status);
	}
	
	return found;
}

/** Remove lost targets
*/
void CentralizedEstimator::removeLostTargets()
{
	auto it = targets_.begin();

	while(it != targets_.end())
	{		
		//TODO: target status is never LOST so this check doesn't make sense now
		if( ((it->second)->getStatus() == LOST) 
		|| ((it->second)->isStatic() == false && (it->second)->lastUpdateTime() > lost_th_ && (it->second)->getUpdateCount() < min_update_count_) 
		|| ((it->second)->isStatic() == true && (it->second)->lastUpdateTime() > lost_th_ && (it->second)->getUpdateCount() < min_update_count_) )
		{
			delete(it->second);
			it = targets_.erase(it);
		}
		else
			++it;
	}
}

/** Print information from the targets for debugging
*/
void CentralizedEstimator::printTargetsInfo()
{
	cout << "****************************************************" << endl;
	cout << "Number of targets: " << targets_.size() << endl;

	for(auto it = targets_.begin(); it != targets_.end(); ++it)
	{
		cout << "Id: " << (it->second)->getId() << ". ";

		switch((it->second)->getStatus())
		{
			case ACTIVE	:
			cout << "Status: " << "ACTIVE. "; 
			break;
			case LOST:
			cout << "Status: " << "LOST. ";
			break;
			default:
			cout << "Status: " << "ERROR. ";
		}

		if( (it->second)->getStatus() == ACTIVE )
		{
			double x, y, vx, vy;
			vector<vector<double> > cov;
			vector<double> size_probs;

			(it->second)->getPose(x,y);
			(it->second)->getVelocity(vx,vy);
			cov = (it->second)->getCov();
			size_probs = (it->second)->getFactorProbs(0);

			cout << "Position: " << x << "," << y << ". Velocity: " << vx << "," << vy << ". Covariances: " << cov[0][0] << " " << cov[0][1] << "; " << cov[1][0] << " " << cov[1][1] << "." << endl;
			cout << "Size: ";
			switch((it->second)->getObjSize())
			{
				case UNKNOWN:
				cout << "UNKNOWN. ";
				break;
				case SMALL:
				cout << "SMALL. ";
				break;
				case MEDIUM:
				cout << "MEDIUM. ";
				break;
				case LARGE:
				cout << "LARGE. ";
				break;
				default:
				cout << "ERROR. ";
			}

			cout << "( ";
			for(int i = 0; i < size_probs.size(); i++)
				cout << size_probs[i] << " ";

			cout << "). ";

			cout << "Static? ";
			if((it->second)->isStatic())
				cout << "yes. ";
			else
				cout << "no. ";
		}
		cout << endl;
	}
}