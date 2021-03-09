%localization analysis
clear all
dy_dlat = 111320
dx_dlon = 83351  %this depends on your latitude; number here is for Cleveland

heading_odom_init = 0.0; %tune this
scale_steering_to_curvature =1; %this is not true; TUNE ME
scale_tics_per_meter = 1; %ALSO not true; TUNE ME

%load a trip file w/ gps and can data:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data = load("slalom_data.csv");
format long
lat_start = data(1,1)
lon_start = data(1,2)
lat_vec = data(:,1); %latitude measurements
lon_vec = data(:,2); %longitude measurements
%convert lat/lon to x,y w/rt starting position
%magic numbers to convert deg lat and deg lon to dy and dx
%NOTE: GPS calls longitude -81deg, not 81deg "West"
%so longitude gets more positive heading east, = x direction
%latitude gets more positive heading north, = y direction
y_vec_gps = (data(:,1)-lat_start)*dy_dlat;
x_vec_gps = (data(:,2)-lon_start)*dx_dlon;
enc = (data(:,4)-data(1,4));  %encoder values are in column 4
%enc_dist = enc/scale_tics_per_meter;  %should convert encoder values to distance w/ scale factor and offset
t_vec = data(:,5);
figure(1)
plot(t_vec,x_vec_gps,t_vec,y_vec_gps)
title('GPS x and y vs time')

[npts,ncols] = size(data);
steering_can_vec = data(:,3); %raw steering data is in column 3

figure(2)
plot(x_vec_gps,y_vec_gps,'b')
title('x vs y per gps ')
axis("equal")
grid on
hold on

%compute odometry based on encoder and steering values:
x_odom_start = x_vec_gps(1);
y_odom_start = y_vec_gps(1);
heading_odom = heading_odom_init;
x_odom_vec = [x_odom_start];
y_odom_vec = [y_odom_start];
heading_odom_vec = [heading_odom];
[npts,dummy] = size(x_vec_gps)
for i=2:npts
  heading_odom_vec(i)=0; %NOT true
  x_odom_vec(i) = 0; %definitely not true
  y_odom_vec(i) = 0; %also not true
end
plot(x_odom_vec,y_odom_vec,'r')


