//------------------------------------------------------------------------------
// GRVC
//------------------------------------------------------------------------------
//
// Copyright (c) 2016 GRVC University of Seville
//
//------------------------------------------------------------------------------

#ifndef CANDIDATE_H_
#define CANDIDATE_H_

#include <Eigen/Eigen>

struct Candidate{
    Eigen::Vector2d location;
    Eigen::Vector2d speed;

    Eigen::Matrix2d location_covariance;
    Eigen::Matrix2d speed_covariance;

    /// Candidate's size. -1:unknown, 0: Small, 1: Medium, 2: Large.
    int size;

};

#endif
