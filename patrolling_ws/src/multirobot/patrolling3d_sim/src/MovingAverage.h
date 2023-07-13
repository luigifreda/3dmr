/**
* This file is part of the ROS package patrolling3d_sim which belongs to the framework 3dpatrolling. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com> (La Sapienza University)
* For more information see <https://github.com/luigifreda/3dmr>
*
* 3DMR is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DMR is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DMR. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <iostream>
#include <vector>


///	\class MovingAverage
///	\author Luigi Freda
///	\brief A class for computing a moving average (mean value) with a given window size. Optionally, you can also compute standard devitation of the signal and standard deviation of the average
///	\note
/// 	\todo 
///	\date
///	\warning
class MovingAverage
{
    static const int kDefaultWidth = 600; // 

public:
    MovingAverage(int averageWidth = kDefaultWidth, bool computeSigma = true);
    ~MovingAverage();

    void Init(double initVal);

    void Reset()
    {
        bInit_ = false;
    }

public: /// getters

    double GetAverageVal() const
    {
        return dAverage_;
    }
    
    double GetCurrentVal() const
    {
        return dCurrentVal_;
    }

    double GetSigma() const
    {
        return sqrt(std::max(dSigma2_, 0.));
    }

    double GetSigmaOfAverage() const
    {
        return sqrt(std::max(dSigma2_, 0.) / iAverageWidth_);
    } /// sigma (std deviation) of the average (mean)
    double GetAverage(double newVal);

    int GetSize() const
    {
        return iAverageWidth_;
    }

protected:
    int iAverageWidth_;
    int iRing_;
    double dAverage_;
    double dSigma2_;
    bool bInit_;
    
    double dCurrentVal_; 

    bool bComputeSigma_;

    double dOneOverAverageWidthMinOne_;

    std::vector<double> ringBuffer_;
};

inline MovingAverage::MovingAverage(int averageWidth, bool computeSigma)
{
    iAverageWidth_ = averageWidth;

    dOneOverAverageWidthMinOne_ = 1. / (averageWidth - 1);

    ringBuffer_.resize(iAverageWidth_, 0);

    iRing_ = 0;
    dAverage_ = 0;
    
    dCurrentVal_ = 0;

    bInit_ = false;

    bComputeSigma_ = computeSigma;
}

inline MovingAverage::~MovingAverage()
{
}

inline double MovingAverage::GetAverage(double newVal)
{
    if (!bInit_) Init(newVal);

    double averageOld = dAverage_;
    double oldVal = ringBuffer_[iRing_];

    dAverage_ += (newVal - oldVal) / iAverageWidth_;

    if (bComputeSigma_)
    {
        dSigma2_ = dSigma2_ + dOneOverAverageWidthMinOne_ * (iAverageWidth_ * (averageOld * averageOld - dAverage_ * dAverage_) - oldVal * oldVal + newVal * newVal);
    }

    ringBuffer_[iRing_] = newVal;
    dCurrentVal_        = newVal; 

    iRing_ = (iRing_ + 1) % iAverageWidth_;

    return dAverage_;
}

inline void MovingAverage::Init(double initVal)
{

    for (int h = 0; h < ringBuffer_.size(); h++)
    {
        ringBuffer_[h] = initVal;
    }
    
    dCurrentVal_ = initVal;

    dAverage_ = initVal;

    dSigma2_ = 0;

    bInit_ = true;
}


#endif
