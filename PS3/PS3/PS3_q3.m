% Challenge question: joint home calibration:
% For the same robot as in Part 1, it is desired to calibraq1, uncalibrated: 0.8, 0.7, 0.1, 0.3, 1.7, 4.5te the (manufacturer’s) home angles. The
% reported joint angles are off by a relatively small amount (but different amount for each joint). It is
% desired to find these joint offsets and correct for them based on theodolite measurements.

% The robot has a precision-mounted target on its toolflange, and a theodolite captures precise poses,
% R_flange/sensor and o_flange/sensor with measurements R1(q1), o1(q1) through R4 (q4)and o4 (q4). That
% is, the robot is sent to poses q1 through q4 , as measured by (uncalibrated) joint encoders. The
% theodolite measures the resulting position and orientation of the flange-mounted marker for each pose.


%% motoman robot specs
%dh_row = zeros(6,4);

% dh_row1 = [0.145, 0.540, 3*pi/2 , q1];
% dh_row2 = [1.150, 0.0, -pi , q2-pi/2];
% dh_row3 = [0.250, 0.0, -pi/2 , q3];
% dh_row4 = [0.0 ,-1.812 ,pi/2 , q4];
% dh_row5 = [0.0 ,0 , 3*pi/2 , q5];
% dh_row6 = [0.0 ,-0.100 ,pi , q6];
% dh_row =[ dh_row1; dh_row2; dh_row3; dh_row4; dh_row5 ; dh_row6];

%% list the poses for the motoman
offset = zeros(1,6);
%offset = [0, 0.1, 0, 0.5, 0.22, 0]  
offset = [-1.2276, -0.5351, -0.8867, 0.0577, 0.3356, 0]   %joseph's

pose1 = [0.8 , 0.7 , 0.1, 0.3 , 1.7 , 4.5]  ;
pose2 = [1, 0.3, 0.5, -0.1, 1, 2 ] ;
pose3 = [-0.5, -0.1, 1, -0.1, 1, 2] ;
pose4 = [1, 0.5, 0.5, 0, 0, 0] ; 

%% find the end point corresponding to each pose wrt base.

A_flange_base_pose1 = find_A_flange_base("motoman",pose1 + offset);
A_flange_base_pose2 = find_A_flange_base("motoman",pose2 + offset);
A_flange_base_pose3 = find_A_flange_base("motoman",pose3 + offset);
A_flange_base_pose4 = find_A_flange_base("motoman",pose4 + offset);


%% R_flange/base and o_flange/base for four poses - derived from above

R_flange_base_pose1 = A_flange_base_pose1(1:3,1:3);
R_flange_base_pose2 = A_flange_base_pose2(1:3,1:3);
R_flange_base_pose3 = A_flange_base_pose3(1:3,1:3);
R_flange_base_pose4 = A_flange_base_pose4(1:3,1:3);

O_flange_base_pose1 = A_flange_base_pose1(1:3,4);
O_flange_base_pose2 = A_flange_base_pose2(1:3,4);
O_flange_base_pose3 = A_flange_base_pose3(1:3,4);
O_flange_base_pose4 = A_flange_base_pose4(1:3,4);

O_flange_base_list = [O_flange_base_pose1';
                      O_flange_base_pose2'; 
                      O_flange_base_pose3';
                      O_flange_base_pose4' ];




%% We have R_flange/sensor and o_flange/sensor given

O_flange_sensor_pose1 = [0.586558, 0.0683277, -1.23548];
O_flange_sensor_pose2 = [0.338239, 0.466427, 0.428277 ];
O_flange_sensor_pose3 = [-2.16733, -0.115077, 1.44734 ];
O_flange_sensor_pose4 = [0.70385, 0.464142, -0.100951 ];

O_flange_sensor_list = [O_flange_sensor_pose1;
                        O_flange_sensor_pose2;
                        O_flange_sensor_pose3; 
                        O_flange_sensor_pose4 ];


R_flange_sensor_pose1 =[
-0.0147655, -0.854004, 0.520057;
0.994544, 0.0411739, 0.0958502;
-0.103269 ,0.518635, 0.848736];

R_flange_sensor_pose2 =[
-0.0147655, -0.854004, 0.520057;
0.994544, 0.0411739, 0.0958502;
-0.103269 ,0.518635, 0.848736];

R_flange_sensor_pose3 =[
-0.0147655, -0.854004, 0.520057;
0.994544, 0.0411739, 0.0958502;
-0.103269 ,0.518635, 0.848736];

R_flange_sensor_pose4 =[
-0.0147655, -0.854004, 0.520057;
0.994544, 0.0411739, 0.0958502;
-0.103269 ,0.518635, 0.848736];

%% Let us assume that the two flange values are pointing to the same point in space wrt to base and sensor
%% Calculate R and O transform matrix using svt and the two origin matrices

A_transform_base_sensor = find_transform_vec_from_points( O_flange_base_list',O_flange_sensor_list');

inv(A_transform_base_sensor);

%% Convert theoretical value to sensor frame
%% R*A +t = B
%% R*/base + t = /sensor
R_transform_base_sensor  = A_transform_base_sensor(1:3,1:3)
O_transform_base_sensor =  A_transform_base_sensor(1:3,4)

theo_sensor1 = R_transform_base_sensor * O_flange_base_pose1 + O_transform_base_sensor;
theo_sensor2 = R_transform_base_sensor * O_flange_base_pose2 + O_transform_base_sensor;
theo_sensor3 = R_transform_base_sensor * O_flange_base_pose3 + O_transform_base_sensor;
theo_sensor4 = R_transform_base_sensor * O_flange_base_pose4 + O_transform_base_sensor;

theo_sensor_list = [theo_sensor1'; theo_sensor2'; theo_sensor3'; theo_sensor4'];
O_flange_sensor_list = [O_flange_sensor_pose1; O_flange_sensor_pose2; O_flange_sensor_pose3; O_flange_sensor_pose4 ];

%% if we assume they are the same point, this diff should be  0
diff = theo_sensor_list - O_flange_sensor_list ;

error = zeros(4,1);
for n = 1 : 4
    error(n,:) = norm(diff(n,:));
end


error = (norm(error'))

    










%% function here:
function A = find_A_flange_base(robot, pose)
    if robot == "motoman"
        dh_params = get_dh_params(pose);
    end
     A10 = find_A_from_DH(dh_params(1,:));% give entire first row
     A21 = find_A_from_DH(dh_params(2,:));
     A32 = find_A_from_DH(dh_params(3,:));
     A43 = find_A_from_DH(dh_params(4,:));
     A54 = find_A_from_DH(dh_params(5,:));
     A65 = find_A_from_DH(dh_params(6,:));
     A =  A10 * A21 * A32 * A43 * A54 * A65;
    
end

function dh_params = get_dh_params(pose)
    dh_row1 = [0.145, 0.540, 3*pi/2 , pose(1)];
    dh_row2 = [1.150, 0.0, -pi , pose(2)-pi/2];
    dh_row3 = [0.250, 0.0, -pi/2 , pose(3)];
    dh_row4 = [0.0 ,-1.812 ,pi/2 , pose(4)];
    dh_row5 = [0.0 ,0 , 3*pi/2 , pose(5)];
    dh_row6 = [0.0 ,-0.100 ,pi , pose(6)];
    dh_params =[ dh_row1; dh_row2; dh_row3; dh_row4; dh_row5 ; dh_row6];
    
end

function [A] = find_A_from_DH(dh_row)
    a = dh_row(1,1);
    d = dh_row(1,2);
    alpha = dh_row(1,3);
    theta = dh_row(1,4);
    
    size(dh_row);
    A = eye(4); % EYE(N) is the N-by-N identity matrix.
    %row 1
    A(1,1) = cos(theta);
    A(1,2) = -sin(theta)* cos(alpha);
    A(1,3) = sin(theta)* sin(alpha);
    A(1,4) = a * cos(theta);
    % row 2
    A(2,1) = sin(theta);
    A(2,2) = cos(theta)* cos(alpha);
    A(2,3) = -cos(theta)* sin(alpha);
    A(2,4) = a * sin(theta);
    % row 3
    A(3,1) = 0;
    A(3,2) = sin(alpha);
    A(3,3) = cos(alpha);
    A(3,4) = d ;
    % row 4
    A(4,1:3) = 0;  
%    A(4,4) = d ;
end
 
function [A_m2]= find_transform_vec_from_points(vec1, vec2)
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
end
