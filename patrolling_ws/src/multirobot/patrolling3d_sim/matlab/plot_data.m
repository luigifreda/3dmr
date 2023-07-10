if ~exist('plot_all')
    clc; close all; clear all; 
end

test_data_folder ='./data'; % create a symbolic link to your data

%test_exp='vrep_crossroad3_with4_4'; 
%test_exp='vrep_3D_ring_4' ; 
test_exp='vrep_ring_tails_4' ;
%test_exp='vrep_3D_simple_blocks_4'; 
%test_exp='vrep_crossroad3_3'; 
%test_exp='vrep_fork4_4'; 
%test_exp='vrep_long_corridor4_4';
%test_exp='vrep_crossroad4_4'; 
%test_exp='vrep_3ways_3'; 
 


%% set input data

max_period=59; % 60 sec * max_period = 3600
 
if ~exist('plot_all')
    experiment  = test_exp
    data_folder = test_data_folder
end

algorithm='CR';
hostname='titan';
%cc_folder='cc'; no_cc_folder='no_cc';
selected_folder='selected';

cc_filename    ='cc_timeresults_moving_avg';
cwc_filename   ='cwc_timeresults_moving_avg';
no_cc_filename ='no_cc_timeresults_moving_avg';

filename_ext='.csv';

%% set output name

out_folder =[data_folder,'/output'];

if not(exist(out_folder,'dir'))
    mkdir(out_folder)
end


filename_idleness_stats_out = [out_folder,'/',experiment,'_idleness_stats_out'];
filename_idleness_max_out   = [out_folder,'/',experiment,'_idleness_max_out'];
filename_interferences_out  = [out_folder,'/',experiment,'_interferences_out'];

out_format='png';


%% load data

cc_path    = [data_folder,'/',experiment,'/',algorithm,'/',hostname,'/',selected_folder,'/',cc_filename,filename_ext]
cwc_path   = [data_folder,'/',experiment,'/',algorithm,'/',hostname,'/',selected_folder,'/',cwc_filename,filename_ext]
no_cc_path = [data_folder,'/',experiment,'/',algorithm,'/',hostname,'/',selected_folder,'/',no_cc_filename,filename_ext]

time_interval = 1:max_period;

if exist(cc_path,'file')
    cc_data    = dlmread(cc_path,';',1);
    cc_time            = cc_data(time_interval,1);
    cc_idleness_min    = cc_data(time_interval,2);
    cc_idleness_avg    = cc_data(time_interval,3);
    cc_idleness_stddev = cc_data(time_interval,4);
    cc_idleness_max    = cc_data(time_interval,5);
    cc_interferences   = cc_data(time_interval,6);
end

if exist(cwc_path,'file')
    cwc_data    = dlmread(cwc_path,';',1);
    cwc_time            = cwc_data(time_interval,1);
    cwc_idleness_min    = cwc_data(time_interval,2);
    cwc_idleness_avg    = cwc_data(time_interval,3);
    cwc_idleness_stddev = cwc_data(time_interval,4);
    cwc_idleness_max    = cwc_data(time_interval,5);
    cwc_interferences   = cwc_data(time_interval,6);
end    

if exist(no_cc_path,'file')
    no_cc_data = dlmread(no_cc_path,';',1);
    no_cc_time            = no_cc_data(time_interval,1);
    no_cc_idleness_min    = no_cc_data(time_interval,2);
    no_cc_idleness_avg    = no_cc_data(time_interval,3);
    no_cc_idleness_stddev = no_cc_data(time_interval,4);
    no_cc_idleness_max    = no_cc_data(time_interval,5);
    no_cc_interferences   = no_cc_data(time_interval,6);
end


%% draw

% comparison cc with no_cc
if exist(cc_path,'file') && exist(no_cc_path,'file')
    h= figure;
    set_plot_style; 
    title('idleness statistics')
    errorbar(cc_time,cc_idleness_avg,cc_idleness_stddev);
    errorbar(no_cc_time,no_cc_idleness_avg,no_cc_idleness_stddev,'Linestyle', '-.');
    xlabel('time [s]');
    ylabel('idleness [s]')
    legend('CC','No CC')
    savefig(filename_idleness_stats_out); 
    saveas(gcf,filename_idleness_stats_out,out_format)
    printpdf(h,[filename_idleness_stats_out,'.pdf'])

    h= figure;
    set_plot_style; 
    title('idleness max')
    plot(cc_time,cc_idleness_max)
    plot(no_cc_time,no_cc_idleness_max,'-.')
    xlabel('time [s]');
    ylabel('idleness [s]')
    legend('CC','No CC')
    savefig(filename_idleness_max_out); 
    saveas(gcf,filename_idleness_max_out,out_format)
    printpdf(h,[filename_idleness_max_out,'.pdf'])

    h= figure;
    set_plot_style; 
    title('interferences')
    plot(cc_time,cc_interferences)
    plot(no_cc_time,no_cc_interferences,'-.')
    xlabel('time [s]');
    ylabel('#inteferences')
    legend('CC','No CC')
    savefig(filename_interferences_out); 
    saveas(gcf,filename_interferences_out,out_format)
    printpdf(h,[filename_interferences_out,'.pdf'])
end

% comparison cc with cc_no_metric
if exist(cc_path,'file') && exist(cwc_path,'file')
    h= figure;
    set_plot_style; 
    title('idleness statistics')
    errorbar(cc_time,cc_idleness_avg,cc_idleness_stddev);
    errorbar(cwc_time,cwc_idleness_avg,cwc_idleness_stddev,'Linestyle', '-.');
    xlabel('time [s]');
    ylabel('idleness [s]')
    legend('CC','CwMC')
    savefig(filename_idleness_stats_out); 
    saveas(gcf,filename_idleness_stats_out,out_format)
    printpdf(h,[filename_idleness_stats_out,'.pdf'])

    h= figure;
    set_plot_style; 
    title('idleness max')
    plot(cc_time,cc_idleness_max)
    plot(cwc_time,cwc_idleness_max,'-.')
    xlabel('time [s]');
    ylabel('idleness [s]')
    legend('CC','CwMC')
    savefig(filename_idleness_max_out); 
    saveas(gcf,filename_idleness_max_out,out_format)
    printpdf(h,[filename_idleness_max_out,'.pdf'])

    h= figure;
    set_plot_style; 
    title('interferences')
    plot(cc_time,cc_interferences)
    plot(cwc_time,cwc_interferences,'-.')
    xlabel('time [s]');
    ylabel('#inteferences')
    legend('CC','CwMC')
    savefig(filename_interferences_out); 
    saveas(gcf,filename_interferences_out,out_format)
    printpdf(h,[filename_interferences_out,'.pdf'])
end
