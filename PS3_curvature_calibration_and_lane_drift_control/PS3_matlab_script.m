% Nicole Graf, Joseph Cressman, and Andrew Capelli
% PS3: 
% Due 24 February 2021
% close all; clear all; clc

r_earth = 6.371e6;
lat_to_meters = r_earth*pi/180;
lon_to_meters_at_41degN = r_earth*cos(41*pi/180)*pi/180;
%% Reading the CSVs Nicole's way
figure;
for i = 1:15 %update to reflect names of data
    filename = ['trimdcircle', (num2str(i)), '.csv'];
    data = csvread(filename);
    % syntax for getting data
    lat_start = data(1,1);
    lon_start = data(1,2);
    
    % coordinates in meters
    y = (data(:,1)-lat_start)*lat_to_meters;
    x = (data(:,2)-lon_start)*lon_to_meters_at_41degN;
    
    n = size(y,1);
    steering_angle = data(:,3);
    
    % 1st and 2nd derivatives of x and y
    dx = diff(x);
    dy = diff(y);
    d2x = diff(dx);
    d2y = diff(dy);
    
    % Adjust size of 1st derivatives to match 2nd
    dx = (dx(1:n-2)+dx(2:n-1))/2;
    dy = (dy(1:n-2)+dy(2:n-1))/2;
    
    % Compute curvature
    curvature = (dx.*d2y-dy.*d2x)./(dx.^2+dy.^2).^(3/2);
    
    %Resize steering angle to match
    sa = steering_angle;
    sa = (sa(1:n-2)+sa(2:n-1)+sa(3:n))/3;
    
    % Plot curvature against steering angle
    plot(sa,curvature,'*')
    hold on
    
    % Plot x,y path of vehicle
%     plot(x,y);
%     hold on
end
