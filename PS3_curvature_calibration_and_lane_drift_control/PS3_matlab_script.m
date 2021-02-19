% Nicole Graf, Joseph Cressman, and Andrew Capelli
% PS3: 
% Due 24 February 2021
close all; clear all; clc
%% Reading the CSVs
% Code from Newman
data = load("circle15.csv");
lat_start = data(1,1) %for convenience, compute x,y motions relative to starting location 
lon_start = data(1,2)
    % convert lat/lon to x,y w/rt starting position
    % magic numbers to convert deg lat and deg lon to dy and dx
    % NOTE: GPS calls longitude -81deg, not 81deg "West"
    % so longitude gets more positive heading east, = x direction
    % latitude gets more positive heading north, = y direction
% y_vec = (data(:,1)-lat_start)*lat_to_meters;
% x_vec = (data(:,2)-lon_start)*lon_to_meters_at_41degN;

%% Reading the CSVs Nicole's way
all_lat = [];
all_lon = [];
all_steering_angle = [];
 for i = 1:1:15 %update to reflect names of data
    filename = ['circle', (num2str(i)), '.csv'];
    data = csvread(filename);
    % syntax for getting data
    lat_start = data(:,1);
    lon_start = data(:,2);
    steering_angle = data(:,3);
    
    % generating plots of the data (all 15 datas)
    % figure(i)
    % clf(i)
    % figure(i)
hold on
    % plot(time,load); Save this for later... put plots here
    %all_lat = [all_lat, lat_start];
    %all_lon = [all_lon, lon_start];
    %all_steering_angle = [all_steering_angle, steering_angle];
% save jpeg syntax save as filename
    % picturename = ['data', num2str(i), '.jpeg'];
    % saveas (figure(i),picturename)
 end
%  all_lats' 
%  pause
%  all_lon'
%  pause
%  all_steering_angle'