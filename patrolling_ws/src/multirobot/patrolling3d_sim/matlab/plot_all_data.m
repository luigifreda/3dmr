clc; close all; clear all; 

plot_all = 1; 

%% set input data

data_folder='./data';

experiment_list = dir(data_folder)
num_folders     = size(experiment_list,1);

for i=3:num_folders
    experiment = experiment_list(i).name
    plot_data; 
end
