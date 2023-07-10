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

#pragma once 

#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

#include "SignalUtils.h"

///	\class PathSmoother
///	\author Luigi Freda
///	\brief 
///	\note
/// \todo 
///	\date
///	\warning
class PathSmoother
{
public:
    const static double kSavitzkyGolayKernelW5[5];
    const static double kSavitzkyGolayKernelW7[7];

    const static double kPathSmoothingKernel3[3];
    const static double kPathSmoothingKernel5[5];

    enum PathSmootherType
    {
        kNoSmoother = 0, 
        kSmoother3 = 1,
        kSmoother5 = 2,
        kSmoother3InPlace = 3,
        kSmootherSGw5 = 4,
        kSmootherSGw7 = 5,
        kNumOfSmoothers,
    }; 
    const static std::string kPathSmootherTypeStr[kNumOfSmoothers]; 

public:
    
    PathSmoother(){}

public: // getters
    

    static nav_msgs::Path conv(const nav_msgs::Path& path, const double kernel[], const size_t kernel_size)
    {
        const size_t kernel_offset = floor(kernel_size/2);

        nav_msgs::Path path_out = path;  
        const size_t num_points = path.poses.size();

        // first and last points are not changed 
        for (int i = 1; i < num_points-1; i++)
        {
            double& x = path_out.poses[i].pose.position.x; 
            double& y = path_out.poses[i].pose.position.y; 
            double& z = path_out.poses[i].pose.position.z;
            x = 0;
            y = 0;
            z = 0;  

            for(int jj=0; jj<kernel_size; jj++)
            {
                int pj = (i - kernel_offset) + jj; // index of the point to multiply with the kernel element jj (the kernel mask is centered around the current point)
                
                // check if we are out of bounds 
                if(pj<0) pj = 0; // padding: repeat the first element if we are out of bound on the left 
                if(pj>num_points-1) pj = num_points-1; // padding: repeat the last element if we are out of bound on the right

                x += kernel[jj] * path.poses[pj].pose.position.x;
                y += kernel[jj] * path.poses[pj].pose.position.y;
                z += kernel[jj] * path.poses[pj].pose.position.z;
            } 
        }

        return path_out; 
    }

    // smooth the path with a Gaussian kernel of size 3
    static nav_msgs::Path smoothPath3(const nav_msgs::Path& path)
    {
        nav_msgs::Path path_out = path;  
        const size_t num_points = path.poses.size();
        for (int i = 1; i < num_points-1; i++)
        {
            path_out.poses[i].pose.position.x = kPathSmoothingKernel3[0] * path.poses[i-1].pose.position.x 
                                              + kPathSmoothingKernel3[1] * path.poses[i].pose.position.x 
                                              + kPathSmoothingKernel3[2] * path.poses[i+1].pose.position.x;
            path_out.poses[i].pose.position.y = kPathSmoothingKernel3[0] * path.poses[i-1].pose.position.y 
                                              + kPathSmoothingKernel3[1] * path.poses[i].pose.position.y 
                                              + kPathSmoothingKernel3[2] * path.poses[i+1].pose.position.y;
            path_out.poses[i].pose.position.z = kPathSmoothingKernel3[0] * path.poses[i-1].pose.position.z 
                                              + kPathSmoothingKernel3[1] * path.poses[i].pose.position.z 
                                              + kPathSmoothingKernel3[2] * path.poses[i+1].pose.position.z;
        }
        return path_out; 
    }

    // smooth the path in place with a Guassian kernel of size 3
    static nav_msgs::Path smoothPath3InPlace(const nav_msgs::Path& path) // similar to convolving with a longer kernel (not centered at the current position)
    {
        nav_msgs::Path path_out = path;  
        const size_t num_points = path.poses.size();
        for (int i = 1; i < num_points-1; i++)
        {
            // here we re-use the path_out input on the right side 
            path_out.poses[i].pose.position.x = kPathSmoothingKernel3[0] * path_out.poses[i - 1].pose.position.x 
                                              + kPathSmoothingKernel3[1] * path_out.poses[i].pose.position.x 
                                              + kPathSmoothingKernel3[2] * path_out.poses[i + 1].pose.position.x;
            path_out.poses[i].pose.position.y = kPathSmoothingKernel3[0] * path_out.poses[i - 1].pose.position.y 
                                              + kPathSmoothingKernel3[1] * path_out.poses[i].pose.position.y 
                                              + kPathSmoothingKernel3[2] * path_out.poses[i + 1].pose.position.y;
            path_out.poses[i].pose.position.z = kPathSmoothingKernel3[0] * path_out.poses[i - 1].pose.position.z 
                                              + kPathSmoothingKernel3[1] * path_out.poses[i].pose.position.z 
                                              + kPathSmoothingKernel3[2] * path_out.poses[i + 1].pose.position.z;
        }
        return path_out;
    }

    // smooth the path with a Gaussian kernel of size 5
    static nav_msgs::Path smoothPath5(const nav_msgs::Path& path)
    {
        return conv(path, kPathSmoothingKernel5, 5);   
    }

    // smooth the path with a Savitzky-Golay kernel of size 5
    static nav_msgs::Path smoothPathSGw5(const nav_msgs::Path& path)
    {
        return conv(path, kSavitzkyGolayKernelW5, 5);   
    }

    // smooth the path with a Savitzky-Golay kernel of size 7
    static nav_msgs::Path smoothPathSGw7(const nav_msgs::Path& path)
    {
        return conv(path, kSavitzkyGolayKernelW7, 7);   
    }

    static nav_msgs::Path smooth(const nav_msgs::Path& path_in, const PathSmootherType smoother_type)
    {
        nav_msgs::Path path_smoothed; 
        switch(smoother_type)
        {
            case kSmoother3:
                path_smoothed = smoothPath3(path_in);
                break;  
            case kSmoother5:
                path_smoothed = smoothPath5(path_in);
                break; 
            case kSmoother3InPlace:
                path_smoothed = smoothPath3InPlace(path_in);
                break; 
            case kSmootherSGw5:
                path_smoothed = smoothPathSGw5(path_in);
                break; 
            case kSmootherSGw7:
                path_smoothed = smoothPathSGw7(path_in);
                break; 
            case kNoSmoother:   
            default: 
                path_smoothed = path_in;
        }
        return path_smoothed;
    }

};