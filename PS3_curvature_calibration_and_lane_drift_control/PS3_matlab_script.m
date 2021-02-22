% Nicole Graf, Joseph Cressman, and Andrew Capelli
% PS3: Steering Calibration and Lane Drift Control
% Due 24 February 2021

%% Part 1: Curvature Calculation
close all; clear all; clc % Ensures a clean working branch between runs
    % This assignment includes multiple CSV files. 
    % These were obtained with an instrumented Ford F-550 using 
    % a GPS unit with differential corrections, a shaft-mounted 
    % encoder, and readings of steering angle from the vehicles 
    % CAN-bus.
    
    % The files ?circle1.csv? through ?circle15.csv? contain logs
    % of latitude (first column), longitude (second column) and a
    % value monotonically related to steering angle (third 
    % column). For these files, you can ignore the remaining 
    % columns.
    
% Initializes vectors for all the steering and curve inputs
allcurv = [];
allsteer= [];

% Compute the constants to convert between degrees lat/lon to meters x/y
r_earth = 6.371e6;
lat_to_meters = r_earth * pi/180;
    % For motions of lattitude, there is (to a good 
    % approximation) a constant that relates meters/degree 
    % latitude. However, the conversion of meters/degree 
    % longitude depends on the the latitude. Assume a latitude of
    % 41.5 deg North. Find the conversion constants.
lon_to_meters_at_41degN = r_earth * cos(41.5 * pi/180) * pi/180;

% Reading the CSVs Nicole's way
    % For the files circle1.csv through circle15.csv, the 
    % steering angle was deliberately held approximately constant
    % as the vehicle moved in circles while logging GPS data.
    
    % We altered the original circle#.csv to trimdcircle#.csv where all of
    % the repeated rows were removed.

% The following loop runs through all 15 csv files 
for i = 1:15 % update to reflect names of data
        % The vehicle?s on-board diagnostic (OBD) port provides
        % a digital value related to the steering angle, values
        % of which appear in column 3 of the logs.
    filename = ['trimdcircle', (num2str(i)), '.csv'];
    data = csvread(filename);
    % syntax for getting data
    lat_start = data(1,1);
    lon_start = data(1,2);
        % Define a coordinate system that has the x-axis pointing
        % East, the y-axis pointing North, and heading defined as
        % 0 rad pointing East with positive rotation pointing ?up?
        % (e.g., heading North is +pi/2 radians). It will be 
        % convenient to analyze the data from an initial 
        % reference point of (x,y) = (0,0), i.e. by subtracting 
        % off the initial lat/lon.
    y = (data(:,1)-lat_start)*lat_to_meters; % y axis pointing north
    x = (data(:,2)-lon_start)*lon_to_meters_at_41degN; % x axis pointing east
    
    n = size(y,1);
    steering_angle = data(:,3);

    % 1st & 2nd derivatives of x and y
    dx = diff(x);
    dy = diff(y);
    d2x = diff(dx);
    d2y = diff(dy);
    
    % Resize 1st derivative
    dx = (dx(1:n-2)+dx(2:n-1))/2;
    dy = (dy(1:n-2)+dy(2:n-1))/2;
    
    % Compute the curvature
    curvature = (dx.*d2y-dy.*d2x)./(dx.^2+dy.^2).^(3/2);
    
    % Resize the steering angle vector
    steering_angle = (steering_angle(1:n-2)+steering_angle(2:n-1)+steering_angle(3:n))/3;
    
    
    win = 50; % Width of window function
    
    % Smooth out the noisy data
    curvature = conv(curvature,ones(win,1)/win,'same');
    steering_angle = conv(steering_angle,ones(win,1)/win,'same');
    
    % Remove remaining outliers:
    steering_angle(abs(curvature)>0.25) = [];
    curvature(abs(curvature)>0.25) = [];
    
%     plot(steering_angle,curvature,'o')
%     hold on
    % Populates all curve and steering data into the appropriate vectors
    allcurv = [allcurv; curvature];
    allsteer= [allsteer; steering_angle];
%     plot(x,y);
%     hold on
end

figure(1)
clf(1)
figure(1)
plot(allsteer,allcurv,'o');
hold on
reg = polyfit(allsteer,allcurv,1);
sa = min(allsteer):100:max(allsteer);
plot(sa,polyval(reg,sa));
title('Part 1: Steering Value Related to the Steering Angle')
ylabel('meters') % check that this is actually meters
xlabel('radians') % check that this is actually radians

%% Part 2: Lane-Drift Controller
% As we discussed in class, analyze the response of a lane-drift
% controller using a linear control algorithm. Assume you are 
% able to command a curvature, rho(lateral_offset_err, 
% heading_err), and this control algorithm will be formulated as:
rho = -K_offset * d_offset - K_psi * psi;
% Note that both rho and d_offset are signed. Drifting into the 
% left lane is defined as positive offset, and rotating 
% counterclockwise is considered positive curvature.

% The vehicle dynamics is described as: v_x = speed * cos(psi)
v_y = speed * sin(psi);
omega_z = speed * rho; % (= d psi/dt)

% Choose values for K_psi and K_offset. 
    % Describe recommendations for controller gains.
% Choose values for initial offset and heading errors. 
    % Describe influences of initial conditions. 
% Simulate (and plot): x(t), y(t), psi(t) and the path x vs y.

