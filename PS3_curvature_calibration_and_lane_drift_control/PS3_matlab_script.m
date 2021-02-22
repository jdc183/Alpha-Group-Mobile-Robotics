% Nicole Graf, Joseph Cressman, and Andrew Capelli
% PS3: 
% Due 24 February 2021

allcurv = [];
allsteer= [];

%Compute the constants to convert between degrees lat/lon to meters x/y
r_earth = 6.371e6;
lat_to_meters = r_earth*pi/180;
lon_to_meters_at_41degN = r_earth*cos(41*pi/180)*pi/180;
%% Reading the CSVs Nicole's way
for i = 1:15 %update to reflect names of data
    filename = ['trimdcircle', (num2str(i)), '.csv'];
    data = csvread(filename);
    % syntax for getting data
    lat_start = data(1,1);
    lon_start = data(1,2);
    
    y = (data(:,1)-lat_start)*lat_to_meters;
    x = (data(:,2)-lon_start)*lon_to_meters_at_41degN;
    
    n = size(y,1);
    steering_angle = data(:,3);

    % 1st & 2nd derivatives of x and y
    dx = diff(x);
    dy = diff(y);
    d2x = diff(dx);
    d2y = diff(dy);
    
    %Resize 1st derivative
    dx = (dx(1:n-2)+dx(2:n-1))/2;
    dy = (dy(1:n-2)+dy(2:n-1))/2;
    
    % Compute the curvature
    curvature = (dx.*d2y-dy.*d2x)./(dx.^2+dy.^2).^(3/2);
    
    % Resize the steering angle vector
    steering_angle = (steering_angle(1:n-2)+steering_angle(2:n-1)+steering_angle(3:n))/3;
    
    
    win=50;%Width of window function
    
    %Smooth out the noisy data
    curvature = conv(curvature,ones(win,1)/win,'same');
    steering_angle = conv(steering_angle,ones(win,1)/win,'same');
    
    %remove remaining outliers:
    steering_angle(abs(curvature)>0.25) = [];
    curvature(abs(curvature)>0.25) = [];
    
%     plot(steering_angle,curvature,'o')
%     hold on
    
    allcurv = [allcurv; curvature];
    allsteer= [allsteer; steering_angle];
%     plot(x,y);
%     hold on
end
figure
plot(allsteer,allcurv,'o');
hold on
reg = polyfit(allsteer,allcurv,1);
sa = min(allsteer):100:max(allsteer);
plot(sa,polyval(reg,sa));

