%% PS1_Challenge
%% Import options
%import 
lidarPts = csvread('image_key_pts_metric.csv');
cameraPixels = csvread('image_key_pts_pixels.csv');
%transpose to get 20 column vectors
lidarPts = lidarPts';
cameraPixels = cameraPixels';
%calculate camera pts from the intrinsic camera parameters
cameraPts = nan(size(cameraPixels));
z=3;
cameraPts(1,:) = ((cameraPixels(1,:)-1850)/1638)*z; 
cameraPts(2,:) = ((cameraPixels(2,:)-1000)/1638)*z;
cameraPts(3,:) = 3;
%set z-coordinate of both
lidarPts(3,:) = 0;  %from PSET
%% Solve using SVD
%compute average vectors
camera_av = mean(cameraPts,2);
lidar_av = mean(lidarPts,2);
%compute center vectors (aka distances from average)
camera_diffs = cameraPts - camera_av;
lidar_diffs = lidarPts - lidar_av;
%calculate covariance matrix
H = camera_diffs*lidar_diffs';
%calculate SVD
[U,~,V] = svd(H);
V(:,3)=V(:,3).*-1;
R = V*U';   %final rotation matrix
%find origin pts.
t = -R*camera_av + lidar_av;
%final transformation matrix
A = [R t; 0 0 0 1];

