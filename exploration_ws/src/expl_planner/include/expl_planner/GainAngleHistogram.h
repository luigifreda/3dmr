/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com>
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

#ifndef GAIN_ANGLE_HISTOGRAM_H_
#define GAIN_ANGLE_HISTOGRAM_H_

#include <math.h>
#include <Eigen/StdVector>

#define DEBUG_GAIN_ANGLE_HIST 0

///\class GainAngleHistogram 
///\brief 
///\author Luigi Freda
class GainAngleHistogram
{
public:
    GainAngleHistogram(size_t numBins = 20, double sensor_angle = 120*M_PI/180.):numBins_(numBins),sensor_angle_(sensor_angle)
    {
        elementWidthInv_ = 1./(2*M_PI/numBins);
        binGain_.resize(numBins_,0); // each bin represent an orientation
        //numSamples_.resize(numBins_,0);
        
        num_bins_x_half_scan_ = floor(0.5*sensor_angle_*elementWidthInv_);
    }

    void add(double angle, double val)
    {
        const int bin = angleBin(angle);
        //numSamples_[bin]++;
        binGain_[bin] += val; 
    }
    
    int angleBin(double angle)
    {
        return floor(wrapPos(angle)*elementWidthInv_);
    }
    
    double getBestOrientation()
    {
        std::vector<double> orientation_total_gain(numBins_,0); // we have a different total gain for each orientation 
        for(size_t j=0; j<numBins_; j++)
        {
            for(int h=-num_bins_x_half_scan_; h<=num_bins_x_half_scan_; h++)
            {
                const int binK = posMod(j+h,numBins_);
#if DEBUG_GAIN_ANGLE_HIST                
                if(binK < 0 || binK >= numBins_ )
                {
                    std::cout << "GainAngleHistogram::getBestOrientation() - out " << std::endl; 
                    std::cout << " j:  " << j << ", h: " << h << ", n: " << numBins_ <<  ", out: " << posMod(j+h,numBins_) << std::endl;                     
                    quick_exit(-1);
                }
#endif                 
                orientation_total_gain[j] += binGain_[binK];
            }
        }
        
        double bestGain = -std::numeric_limits<double>::max();
        int best_j = 0; 
        for(size_t j=0; j<numBins_; j++)
        {
            if(orientation_total_gain[j] > bestGain )
            {
                best_j = j; 
                bestGain = orientation_total_gain[j];
            }
        }
        return best_j/elementWidthInv_; // convert the index to an angle
    }
    
protected:
    
    // positive modulus: similar to matlab's mod(), result is always positive. not similar to fmod()
    // es: posMod(-3,4)= 1   fmod(-3,4)= -3
    //     posMod(-5,4)= 3   fmod(-5,4)= -1
    inline double posMod(double x, double y)
    {
        return x - y * floor(x / y);
    }
    
    // wraps an angle [rad] so that it is contained in [0,2*M_PI)
    inline double wrapPos(double ang)
    {
        return posMod(ang, 2 * M_PI);
    }    
    
protected:
            
    std::vector<double> binGain_;  // each bin represent an orientation with its gain 
    //std::vector<int> numSamples_;
    size_t numBins_;
    double elementWidthInv_;
    double sensor_angle_;
    int num_bins_x_half_scan_; 
};



#endif 