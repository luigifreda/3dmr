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

#include <limits>
#include "PathSmoother.h"


// n: 3, sigma: 1.3
// weights: [0.274068619061197, 0.45186276187760605, 0.274068619061197]
const double PathSmoother::kPathSmoothingKernel3[3] = {0.3, 0.4, 0.3};

// n: 5, sigma: 1.3
// weights: [0.09877498815325955, 0.239947075640058, 0.3225558724133649, 0.239947075640058, 0.09877498815325955]
const double PathSmoother::kPathSmoothingKernel5[5] = {0.09877498815325955, 0.239947075640058, 0.3225558724133649, 0.239947075640058, 0.09877498815325955};

// https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter#Tables_of_selected_convolution_coefficients 
const double PathSmoother::kSavitzkyGolayKernelW5[5] = {-3./35, 12./35, 17./35, 12./35, -3./35};
const double PathSmoother::kSavitzkyGolayKernelW7[7] = {-2./21, 3./21, 6./21, 7./21, 6./21, 3./21, -2./21};

const std::string PathSmoother::kPathSmootherTypeStr[PathSmoother::kNumOfSmoothers] = {
"kNoSmoother", 
"kSmoother3",
"kSmoother5",
"kSmoother3InPlace",
"kSmootherSGw5",
"kSmootherSGw7" 
};
