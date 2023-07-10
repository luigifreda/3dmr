#!/usr/bin/env python3

# /**
# * This file is part of the ROS package trajectory_control which belongs to the framework 3DPATROLLING. 
# *
# * Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> and Alcor Lab (La Sapienza University)
# * For more information see <https://gitlab.com/luigifreda/3dpatrolling>
# *
# * 3DPATROLLING is free software: you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation, either version 3 of the License, or
# * (at your option) any later version.
# *
# * 3DPATROLLING is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with 3DPATROLLING. If not, see <http://www.gnu.org/licenses/>.
# */

try:
    import sys
    import argparse
    import os
    import subprocess
    import io 
    import signal 
    import datetime
    import csv
    import numpy as np 
    import matplotlib.pyplot as plt
    import time
    import socket
except ImportError as e:
    print(f"[ERROR] in {__file__} " + str(e))
    sys.exit(1)

kVerbose = False  # Set this to True for debugging
    
# set matplotlib font size     
SMALL_SIZE = 10
MEDIUM_SIZE = 10
BIGGER_SIZE = 12

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

    
def parse_csv(file_path):
    with open(file_path) as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=',')
        line_count = 0
        names = None
        data = {}
        data['filename'] = os.path.basename(file_path)
        for row in csv_reader:
            if line_count == 0:
                names = row
                for i in range(0,len(names)):
                    if i == 0:
                        names[0] = names[0][1:].strip()
                    else:
                        names[i] = names[i].strip() 
                print(f'names: {names}')
            else: 
                #print(row)
                assert(len(names) == len(row)) 
                for i in range(0,len(row)):
                    if line_count == 1:
                        data[names[i]] = []
                    #print(names[i] + ' ' + row[i])
                    data[names[i]].append(row[i])         
            line_count += 1
        data['keys'] = names 
        return data 
    return None

    
# Save figure with nice margin
def save_fig(figure, name, dpi = 300, bbox_inches = 0, pad_inches = 0.1):
    # mng = plt.get_current_fig_manager()
    # mng.full_screen_toggle()
    print(f'saving figure: {name}')
    figure.set_size_inches(24, 18)    
    figure.savefig(name, dpi = dpi, bbox_inches = bbox_inches, pad_inches = pad_inches, orientation='landscape')
    
def draw_path(ax, path, color, marker, label, dashed=False):     
    path_x = [x[0] for x in path]
    path_y = [x[1] for x in path]
    if dashed: 
        ax.plot(path_x, path_y, c=color, marker=marker, label=label, linestyle='dashed')   
    else: 
        ax.plot(path_x, path_y, c=color, marker=marker, label=label) 
        
def draw_data(data, args): 
    assert(data is not None) 
    
    markersize = 3
    
    # comment = f'# x, y '  

    data_to_draw = ['timestamp','x', 'y']

    filename = data['filename']
    
    names = data['keys']
    timestamps = np.array(data['timestamp'], dtype=float)
    t0 = timestamps[0]
    timestamps = np.subtract(timestamps, t0)
    
    x = np.array(data['x'], dtype=float)
    y = np.array(data['y'], dtype=float)

    #print(f'timestamps size: ', len(timestamps))
        
    fig_cartesian, ax_cartesian = plt.subplots(1)        
    fig_components, [ax_x, ax_y] = plt.subplots(2)
    
    # set big figures (otherwise tight_layout() won't work below)
    fig_size_inches = (24, 18)
    fig_cartesian.set_size_inches(fig_size_inches)
    fig_components.set_size_inches(fig_size_inches)

    suptitle_fontsize = 12
    subfig_title_fontsize = 8
    subfig_xlabel_fontsize = 8

    fig_cartesian.suptitle('Data from: ' + filename, fontsize=suptitle_fontsize)
    fig_components.suptitle('Data from: ' + filename, fontsize=suptitle_fontsize)

    ax_cartesian.set_title('x-y plane', fontsize=subfig_title_fontsize)
    ax_x.set_title('x component', fontsize=subfig_title_fontsize)
    ax_y.set_title('x component', fontsize=subfig_title_fontsize)

    ax_cartesian.plot(x, y, c='blue', marker='.', label='path') 
    ax_x.plot(timestamps, x, c='blue', marker='.', label='x')
    ax_y.plot(timestamps, y, c='blue', marker='.', label='y')
    
    ax_cartesian.grid(True)		
    ax_cartesian.axis('equal')
    ax_cartesian.set_xlabel('m', fontsize=subfig_xlabel_fontsize)
    ax_cartesian.set_ylabel('m')       
    
    ax_x.set_xlabel('time', fontsize=subfig_xlabel_fontsize)
    ax_x.set_ylabel('m')            
    ax_x.legend(loc='upper right', frameon=False)
    ax_x.grid(True)
    ax_x.set_xticks(np.arange(min(timestamps), max(timestamps)+1, args.tick))
    
    ax_y.set_xlabel('time', fontsize=subfig_xlabel_fontsize)
    ax_y.set_ylabel('m')            
    ax_y.legend(loc='upper right', frameon=False)
    ax_y.grid(True)
    ax_y.set_xticks(np.arange(min(timestamps), max(timestamps)+1, args.tick))

    # NOTE: as for tight_layout(), you may provide an optional rect parameter, which specifies the bounding box 
    # that the subplots will be fit inside. The coordinates must be in normalized figure coordinates and 
    # the default is (0, 0, 1, 1). From https://matplotlib.org/2.0.2/users/tight_layout_guide.html 
    layout_rect_delta = 0.03
    layout_rect = [layout_rect_delta, layout_rect_delta, (1.0-layout_rect_delta), (1.0-layout_rect_delta)]
    layout_pad = 5
    #fig.tight_layout()  # avoid text overlapping
    fig_cartesian.tight_layout(rect=layout_rect, pad=layout_pad)   # avoid text overlapping 
    fig_components.tight_layout(rect=layout_rect, pad=layout_pad)   # avoid text overlapping 
    if args.save_plots: 
        folder_name = os.path.dirname(os.path.abspath(args.input_file))
        path = os.path.join(folder_name, filename)
        save_fig(fig_cartesian, f'{path}_cartesian_plot.png')
        save_fig(fig_components, f'{path}_components_plot.png')
    plt.show()


def signal_handler(sig, frame):
    print('\n{} intercepted Ctrl+C!'.format(os.path.basename(__file__)))
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

        
if __name__ == '__main__':
    
    argparser = argparse.ArgumentParser(description='Parse and draw a log')
    argparser.add_argument("-i", "--input-file", type=str, default=None, help="Path of the input file we want to draw.")
    argparser.add_argument("-t", "--tick", default=0.5, help="Horizontal tick on the time axis.")
    argparser.add_argument("-s", "--save-plots", default=False, action='store_true', help="Save the plots in case you give an input file.")
    args = argparser.parse_args()
    
    # check if we have to draw an input file 
    if args.input_file: 
        print('parsing and drawing input file ' + args.input_file)
        data = parse_csv(args.input_file)
        #print('data: ' + str(data))
        draw_data(data, args)