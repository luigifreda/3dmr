/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DPATROLLING. 
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> 
* For more information see <https://gitlab.com/luigifreda/3dpatrolling>
*
* 3DPATROLLING is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DPATROLLING is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DPATROLLING. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LOW_PASS_FILTERS_H
#define LOW_PASS_FILTERS_H


#include <iostream>
#include <math.h>
#include <time.h>
#include <string.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include "SignalUtils.h"

///	\class LowPassFilter
///	\author Luigi Freda
///	\brief Implementation of a second-order low pass filter G(s) = wn^2/ (s^2 + 2*wn*xi*s + wn^2). wn is the natural frequency and xi is the dumping
///	\note
/// 	\todo 
///	\date
///	\warning
class LowPassFilter
{
public:
    LowPassFilter(double Ts /*s*/, double wn/*rad/s*/, double xi = 0.7);
    virtual ~LowPassFilter();

    /// init
    void init(double val);

    /// basic step of the filter 
    virtual double step(double uk);


public: ///getters

    double getVal() const
    {
        return dYk_;
    }

    double getWn() const
    {
        return dWn_;
    }
    
    // just an approximation 
    double getRiseTime() const 
    {
        return 1.8/dWn_;
    }
    
    // just an approximation 
    double getSettlingTime() const 
    {
        return 4.6/(dXi_*dWn_);
    }

public: ///setters

    /// set all parameters 
    void set(double Ts/*s*/, double wn, double xi = 0.7);

    /// set Ts 
    void setTs(double Ts/*s*/);

    /// set natural frequency 
    void setWn(double wn/*rad/s*/);

    /// re-initialize the filter

    void reset()
    {
        bInit_ = false;
    }


protected:

    /// update the filter 
    double update(double x);

protected:

    bool bInit_; /// when true the filter has been initialized

    double dYk_1_, dYk_2_;
    double dUk_1_, dUk_2_;

    double dC1_, dC2_, dC3_, dC4_, dC5_; /// filter coefficents, yk = c1 * yk_1 + c2 * yk_2 + c3*uk + c4*uk_1 + c5*uk_2

    double dTs_; /// sample time 
    double dWn_; /// natural frequency [rad/s]
    double dXi_; /// damping 

    double dYk_; /// last output value
};


///	\class LowPassFilterS1
///	\author Luigi Freda
///	\brief A class implementing a low pass filter of second order for angles
///	\note
/// 	\todo 
///	\date
///	\warning
class LowPassFilterS1 :
public LowPassFilter
{
public:

    LowPassFilterS1(double Ts, double wn, double xi = 0.7) : LowPassFilter(Ts, wn, xi)
    {
    }

    /// basic step of the filter 
    double step(double uk);
};



///	\class LowPassFilter1stOrd
///	\author Luigi Freda
///	\brief A class implementing a low pass filter of first order, emulating G(s)=k/(1+s*tau)
///	\note
/// 	\todo 
///	\date
///	\warning
class LowPassFilter1stOrd
{
public:

    LowPassFilter1stOrd(double k_in, double tau_in, double Tc_in);

    void init(double uk);

    double update(double uk)
    {
        dYk_ = dC0_ * uk + dC1_ * dUk_1_ + dC2_*dYk_1_;
        dUk_1_ = uk;
        dYk_1_ = dYk_;

        return dYk_;
    }

    double step(double uk)
    {
        if (!bInit_) init(uk);

        return update(uk);
    }



public: /// getters

    double getVal() const
    {
        return dYk_;
    }

public: /// setters

    /// re-initialize the filter

    void reset()
    {
        bInit_ = false;
    }

    void set(double k_in, double tau_in, double Tc_in);

private:
    double dC0_, dC1_, dC2_;
    double dUk_1_, dYk_1_;
    double dK_;
    double dTau_;
    double dTc_;

    double dYk_; /// last output value

    bool bInit_;
};


///--------------------------------------------------------------------------------------
/// inlines
///--------------------------------------------------------------------------------------

inline double LowPassFilter::update(double uk)
{

    dYk_ = dC1_ * dYk_1_ + dC2_ * dYk_2_ + dC3_ * uk + dC4_ * dUk_1_ + dC5_*dUk_2_;

    dUk_2_ = dUk_1_;
    dUk_1_ = uk;

    dYk_2_ = dYk_1_;
    dYk_1_ = dYk_;

    return dYk_;
}

inline double LowPassFilter::step(double uk)
{
    if (!bInit_) init(uk);

    return update(uk);
}

inline double LowPassFilterS1::step(double uk)
{
    if (!bInit_) init(uk);

    uk = dYk_ + diffS1(uk, dYk_);

    return update(uk);
}




#endif
