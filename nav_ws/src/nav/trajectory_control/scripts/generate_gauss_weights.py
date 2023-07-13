#!/usr/bin/python3
# -*- coding: utf-8 -*-

# /**
# * This file is part of the ROS package trajectory_control which belongs to the framework 3DMR. 
# *
# * Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com> and Alcor Lab (La Sapienza University)
# * For more information see <https://github.com/luigifreda/3dmr>
# *
# * 3DMR is free software: you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation, either version 3 of the License, or
# * (at your option) any later version.
# *
# * 3DMR is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with 3DMR. If not, see <http://www.gnu.org/licenses/>.
# */

from math import pi, sqrt, exp
import argparse

def gauss(n=11,sigma=1):
    r = range(-int(n/2),int(n/2)+1)
    return [1 / (sigma * sqrt(2*pi)) * exp(-float(x)**2/(2*sigma**2)) for x in r]

def normalize(weights):
    sum_weights = sum(weights)
    return [x/sum_weights for x in weights]


# This script can be used to generate 1-D Gaussian kernels of different sizes
if __name__ == '__main__':
        
    argparser = argparse.ArgumentParser(description='Create normalized 1-D Gaussian weights')

    argparser.add_argument("-n", "--num_elements", type=int, default=3, help="Number of elements of the Guassian weights.")
    argparser.add_argument("-s", "--sigma", type=float, default=1, help="Sigma of the Gaussian kernel")
    args = argparser.parse_args()
    
    
    n=args.num_elements
    sigma=args.sigma 
    gauss_weights = normalize(gauss(n,sigma))
    sum_weights = sum(gauss_weights)
    print(f'n: {n}, sigma: {sigma}')
    print(f'weights: {gauss_weights}')
    print(f'sum: {sum_weights}')