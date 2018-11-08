//------------------------------------------------------------------------------
// GRVC 
// Author Ignacio Perez <iperde@upo.es>
//------------------------------------------------------------------------------
//
// Copyright (c) 2016 GRVC University of Seville
//
//------------------------------------------------------------------------------

#ifndef TIMER_H_
#define TIMER_H_

#include <chrono>

/**
 * class Timer
 *
 * A simple timer implementation by using std::chrono
 *
 * @author Ignacio Perez
 */

class Timer
{
public:
    /**
     * Create a new timer and start it
     */
    Timer() : beg_(clock_::now()) {}
    /**
     * Reset the timer to 0
     */
    void reset() { beg_ = clock_::now(); }
    /**
     * Get the elapsed time in seconds since the creation
     * of the object or the last time reset() method was called
     * @return elapsed: the elapsed time in seconds
     */
    double elapsed() const { 
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }
private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};

#endif