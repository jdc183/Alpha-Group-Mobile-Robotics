%localization analysis
 clear all
% dy_dlat = 111320
% dx_dlon = 83351  %this depends on your latitude; number here is for Cleveland

r_earth = 6.368977e6;
dy_dlat = r_earth * pi/180;
dx_dlon = r_earth * cos(41.4308409033 * pi/180) * pi/180;


scale_steering_to_curvature =1.0e-03*0.023522898041216;%From PS3
scale_steering_to_curvature = scale_steering_to_curvature*1.0632;

%load a trip file w/ gps and can data:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data = load("slalom_data.csv");
[npts,ncols] = size(data);
format long
lat_start = data(1,1);
lon_start = data(1,2);
lat_vec = data(:,1); %latitude measurements
lon_vec = data(:,2); %longitude measurements
steering_can_vec = data(:,3); %raw steering data is in column 3
%convert lat/lon to x,y w/rt starting position
y_vec_gps = (data(:,1)-lat_start)*dy_dlat;
x_vec_gps = (data(:,2)-lon_start)*dx_dlon;

heading_odom_init = atan2(y_vec_gps(700),x_vec_gps(700))*.97; %initial heading from gps

gps_arclength = sum(sqrt(diff(x_vec_gps).^2+diff(y_vec_gps).^2));

enc = (data(:,4)-data(1,4));  %encoder values are in column 4
enc_arclength = enc(npts);

scale_tics_per_meter = enc_arclength/gps_arclength;

enc_dist = enc/scale_tics_per_meter;  %should convert encoder values to distance w/ scale factor and offset
t_vec = data(:,5);
figure(1)
plot(t_vec,x_vec_gps,t_vec,y_vec_gps)
title('GPS x and y vs time')




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
  heading_odom_vec(i)=heading_odom_vec(i-1)+steering_can_vec(i)*scale_steering_to_curvature*(enc_dist(i)-enc_dist(i-1)); %dpsi = kappa*dr
  x_odom_vec(i) = x_odom_vec(i-1)+cos(heading_odom_vec(i))*(enc_dist(i)-enc_dist(i-1)); %dx = dr*cos(psi)
  y_odom_vec(i) = y_odom_vec(i-1)+sin(heading_odom_vec(i))*(enc_dist(i)-enc_dist(i-1)); %dy = dr*sin(psi)
end
plot(x_odom_vec,y_odom_vec,'r')
