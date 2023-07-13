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

from copy import deepcopy
import numpy as np
import matplotlib.pyplot as plt
import math 
import os
import time
import io  

# This script was partly inspired by this tutorial:
# https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4

# =========================================================

def print_paths(path, newpath):
    for old, new in zip(path, newpath):
        print('[' + ', '.join('%.3f' % x for x in old) +
              '] -> [' + ', '.join('%.3f' % x for x in new) + ']')

def optimization_based_smooth(path, weight_data=1.0, weight_smooth=0.8, tolerance=0.000001, learning_rate = 0.01):
    """
    Creates a smooth path for a n-dimensional series of coordinates.
    Optimized function: 
    original path: {x(i)}
    smoothed path: {y(i)}
    Cost({y(i)}) = weight_data*||x(i)-y(i)||^2 + weight_smooth*||y(i-1) - y(i)||^2 + weight_smooth*||y(i+1) - y(i)||^2)
    Gradient_yi = -2*weight_data*(x(i)-y(i)) - weight_smooth*(y(i-1) - y(i)) - weight_smooth*(y(i+1) - y(i)) 
    Arguments:
        path: List containing coordinates of a path
        weight_data: Float, how much weight to update the data (alpha)
        weight_smooth: Float, how much weight to smooth the coordinates (beta).
        tolerance: Float, how much change per iteration is necessary to keep iterating.
    Output:
        new: List containing smoothed coordinates.
    """

    new = deepcopy(path)
    dims = len(path[0])
    change = tolerance
    
    learning_rate = 0.1

    while change >= tolerance:
        change = 0.0
        for i in range(1, len(new) - 1):
            for j in range(dims):

                x_i = path[i][j]
                y_i, y_prev, y_next = new[i][j], new[i - 1][j], new[i + 1][j]

                y_i_saved = y_i
                y_i += learning_rate * ( weight_data * (x_i - y_i) + weight_smooth * (y_next + y_prev - (2 * y_i)) )  # gradient descent: subtract the gradient
                new[i][j] = y_i

                change += abs(y_i - y_i_saved)

    return new

def conv_smooth(path, kernel, in_place=False):
    length = len(path)
    #print(f'path length: {length}') 
    path = np.array(path, dtype='f')
    if in_place: 
        smoothed_path = path
    else: 
        smoothed_path = deepcopy(path)
    kernel = np.array(kernel)
    kernel_size = len(kernel)
    kernel_offset = math.floor(kernel_size/2)
    #print(f'kernel size: {kernel_size}, kernel_offset: {kernel_offset}')
    
    for i in range(length):
        if i == 0 or i == (length-1): 
            # first and last point are not changed 
            smoothed_path[i] = path[i]
        else: 
            new_p = 0 
            # kernel covers elements outside the array (we would need padding)
            for jj in range(kernel_size):
                pj = (i - kernel_offset) + jj # index of the point to multiply with the kernel element jj (the kernel mask is centered around the current point)
                if pj < 0: 
                    pj = 0 # padding: repeat the first element if we are out of bounds on the left
                if pj > length-1:
                    pj = length-1 # padding: repeat the last element if we are out of bounds on the right
                new_p += kernel[jj] * path[pj]
            smoothed_path[i] = new_p
    return smoothed_path

# =========================================================

gauss_kernel3 = (0.3, 0.4, 0.3)
gauss_kernel5 = (0.09877498815325955, 0.239947075640058, 0.3225558724133649, 0.239947075640058, 0.09877498815325955)

def gaussian_smooth3(path, kernel=gauss_kernel3, in_place=False):
    return conv_smooth(path, kernel, in_place) 

def gaussian_smooth5(path, kernel= gauss_kernel5, in_place=False):
    return conv_smooth(path, kernel, in_place) 

# =========================================================

# https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter#Tables_of_selected_convolution_coefficients 
savitzky_golay_kernel_w5 =[-3/35, 12/35, 17/35, 12/35, -3/35]
savitzky_golay_kernel_w7 = [-2/21, 3/21, 6/21, 7/21, 6/21, 3/21, -2/21]

def savitzky_golay_filter_w5(path, kernel=savitzky_golay_kernel_w5, in_place=False):
    return conv_smooth(path, kernel, in_place) 

def savitzky_golay_filter_w7(path, kernel= savitzky_golay_kernel_w7, in_place=False):
    return conv_smooth(path, kernel, in_place) 

# =========================================================

def generate_z_path(t_step=0.1, t_end1=0.3, t_end2=0.8, t_end3=0.3):
    path = []
    t = 0
    L1 = 0
    L2 = 0
    while t < t_end1: 
        p = [0, t]
        t += t_step 
        path.append(p)
        L1 = p[1]
    t = 0 
    while t < t_end2: 
        p = [t, L1]
        t += t_step 
        path.append(p)
        L2 = p[0]
    t = 0
    while t < t_end3: 
        p = [L2, L1 + t]
        t += t_step 
        path.append(p)
    return path 

def generate_sin_path(A=1, T=5, t_step=0.1, t_end=4):
    path = []
    t = 0
    omega = 2* math.pi/T
    while t < t_end: 
        p = [t, A*math.sin(omega*t)]
        t += t_step 
        path.append(p)
    return path 

def add_noise(path, sigma=0.01): 
    path = np.array(path, dtype='f')
    noise = np.random.normal(0, sigma, path.shape)
    #print(f'noise: {noise}')
    path = path + noise 
    return path 

# =========================================================


def draw_path(ax, path, color, marker, label, dashed=False): 
    
    save_path_csv(path, f"path_{label}.csv")
    
    path_x = [x[0] for x in path]
    path_y = [x[1] for x in path]
    if dashed: 
        ax.plot(path_x, path_y, c=color, marker=marker, label=label, linestyle='dashed')   
    else: 
        ax.plot(path_x, path_y, c=color, marker=marker, label=label) 


colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']


# Save figure with nice margin
def save_fig(figure, name, dpi = 300, bbox_inches = 0, pad_inches = 0.1):
    # mng = plt.get_current_fig_manager()
    # mng.full_screen_toggle()
    print(f'saving figure: {name}')
    figure.set_size_inches(24, 18)    
    figure.savefig(name, dpi = dpi, bbox_inches = bbox_inches, pad_inches = pad_inches, orientation='landscape')
    
def save_path_csv(path, filename: str, separator=','):
    
    filename = filename.replace(" ", "") # remove all white spaces
    filename = filename.replace(":", "") # remove all :
    file = io.open(filename, "w", encoding='utf-8')
    
    comment = f'# timestamp, x, y'  
    file.write(comment + '\n')
    
    for i,p in enumerate(path): 
        file.write(f'{i}{separator}{p[0]}{separator}{p[1]} \n')
    file.close()
    
# =========================================================

if __name__ == '__main__':
    
    #path = [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [3, 2], [4, 2], [4, 3], [4, 4]]
    path = generate_sin_path()
    #path = generate_z_path()
    
    sigma = 0.02
    path = add_noise(path, sigma=sigma)
    
    # save input path after adding noise 
    save_path_csv(path, "input_path.csv")
    
    # smooth paths 
    opt_smoothed_path = optimization_based_smooth(path)
    
    gauss_smoothed_path3 = gaussian_smooth3(path)
    gauss_smoothed_path3_in_place = gaussian_smooth3(path, in_place=True)
    gauss_smoothed_path5 = gaussian_smooth5(path)
    gauss_smoothed_path5_in_place = gaussian_smooth5(path, in_place=True)
    
    sg_smoothed_path_w5 = savitzky_golay_filter_w5(path)
    sg_smoothed_path_w7 = savitzky_golay_filter_w7(path)    
    
    #print_paths(path, smoothed_path)

    #fig, [ax_input, ax] = plt.subplots(2)
    fig_input, ax_input = plt.subplots(1)
    fig, ax = plt.subplots(1)
        
    ax_input.grid(True)		
    ax_input.axis('equal')
    ax.grid(True)		
    ax.axis('equal')
    #ax.set_aspect('equal', 'box')
    
    # draw input path
    draw_path(ax_input, path, color=colors[0], marker='.', label=f'input sigma:{sigma}')
    
    # draw input path and smoothed paths together 
    draw_path(ax, path, color=colors[0], marker='.', label=f'input sigma:{sigma}', dashed=True) 
    
    draw_path(ax, opt_smoothed_path, color=colors[1], marker='.', label='Iterative Opt smoothed')
    
    draw_path(ax, gauss_smoothed_path3, color=colors[2], marker='.', label='Gauss w3 smoothed')
    #draw_path(ax, gauss_smoothed_path3_in_place, color=colors[3], marker='.', label='gauss3 smoothed in place')
    draw_path(ax, gauss_smoothed_path5, color=colors[4], marker='.', label='Gauss w5 smoothed')
    #draw_path(ax, gauss_smoothed_path5_in_place, color=colors[5], marker='.', label='gauss5 smoothed in place')
    
    draw_path(ax, sg_smoothed_path_w5, color=colors[5], marker='.', label='SG w5 smoothed')
    draw_path(ax, sg_smoothed_path_w7, color=colors[6], marker='.', label='SG w7 smoothed')
    
    ax_input.legend()
    ax.legend()
    
    current_path = os.getcwd()
    time_str = time.strftime("%Y%m%d-%H%M%S")
    
    # save figures 
    save_fig(fig, f'{current_path}/smoothed_paths_{time_str}.png')
    save_fig(fig_input, f'{current_path}/input_path_{time_str}.png')
    
    plt.show()