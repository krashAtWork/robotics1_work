% import vectors
vec1 = csvread('p1vecs.csv') % points as seen in frame 1
vec2 = csvread('p2vecs.csv') % points as seen in frame 2

%% Method 1 - using pseudo - inverse
vec1_a = vec1;
vec2_a = vec2;
% change to 4x1 to work with translation matrix
vec1_a(4,:) = 1;
vec2_a(4,:) = 1;
% calculate pseudo-inverse
piVec1 = pinv(vec1_a);
%calculate transformation matrix
A_m1 = vec2_a*piVec1;

%% Method 2  - using Least-Squares Rigid Motion Using SVD
% compute average vectors
vec1_av = mean(vec1,2);
vec2_av = mean(vec2,2);
%compute center vectors (aka distances from average)
vec1_diffs = vec1 - vec1_av;
vec2_diffs = vec2 - vec2_av;
%calculate covariance matrix
H = vec1_diffs*vec2_diffs';
%calculate SVD
[U,~,V] = svd(H);
R = V*U';   %final rotation matrix
%find origin pts.
t = -R*vec1_av + vec2_av;
%final transformation matrix
A_m2 = [R t; 0 0 0 1];

%% Check work
m1_diffs = norm(vec2_a - A_m1*vec1_a);
m2_diffs = norm(vec2_a - A_m2*vec1_a);
