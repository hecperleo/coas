//------------------------------------------------------------------------------
// GRVC 
// Author Alejandro Braza Barba <alejandrobrazabarba@gmail.com>
//------------------------------------------------------------------------------
//
// Copyright (c) 2016 GRVC University of Seville
//
//------------------------------------------------------------------------------

#ifndef TIMER_H_
#define TIMER_H_

#include <ros/time.h>

/**
 * class Timer
 *
 * A simple timer implementation by using ros::Time
 *
 * @author Alejandro Braza Barba
 */

class Timer
{
public:
    /**
     * Create a new timer and start it
     */
    Timer() : beg_(ros::Time::now()) {}
    /**
     * Reset the timer to 0
     */
    void reset() { beg_ = ros::Time::now(); }
    /**
     * Get the elapsed time in seconds since the creation
     * of the object or the last time reset() method was called
     * @return elapsed: the elapsed time in seconds
     */
    ros::Duration elapsed() const { 
        return ros::Duration
            (ros::Time::now() - beg_); }
private:
    ros::Time beg_;
};

#endif