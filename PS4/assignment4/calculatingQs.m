%% calculating all the q values :
clc
% q1 = 0;
% q2 = 0;
%% specification :
dh_row1 = [0.145, 0.540, 3*pi/2 , q1];
dh_row2 = [1.150, 0.0, -pi , double(q2 - pi/2)];
dh_row3 = [0.250, 0.0, -pi/2 , q3];
dh_row4 = [0.0 ,-1.812 ,pi/2 , q(4)];
dh_row5 = [0.0 ,0 , 3*pi/2 , q(5)];
dh_row6 = [0.0 ,-0.100 ,pi , q(6)];
dh_row =[ dh_row1; dh_row2; dh_row3; dh_row4; dh_row5 ; dh_row6];
%% finding wrist point

o4_0 = [    1.9293,   -2.0000,	0.2707  ]';
wy_0 = o4_0(2,1);
wx_0 = o4_0(1,1);
wz_0 = o4_0(3,1);


%% the most obvious one is Q1
%% the shoulder angle...


%% since the z1 axis points vertically upwards, rotation about that axis causes the end effector to move in a circular plane, keeping z constrained
q1 = atan2(wy_0, wx_0)
 
%% using the q1 value = > we can obtain 0^A1 (all numericals)
A10 = find_A_from_DH(dh_row(1,:));
A01 = inv(A10);



%% thus we can find o4_1 (wrist point) wrt frame 1 using 1^w4 = 1^ A0 *  0^w4 
o4_0aug = [o4_0; 1 ];
o4_1aug = A01 * o4_0aug; 
% as expected o4_1 has z parameter as  0, because wrist point wrt 1 does
% not change with q1 angle
mago4_1 = norm( o4_1aug( 1:3 ,1 ) ); % extracting magnitude of o4_1

 
%% next we find Q3
%% using an important fact that distance between the shoulder and the end-effector is only a function of q3
%% using hand-calculations  as  shown in paper :
d4 = double(dh_row4(2));
a2 = double(dh_row2(1));
num = mago4_1 - d4^2 + a2^2;
den = 2 * d4 * a2 ;
exp = num / den ;
q3 = asin(exp)
% q3_1 = asin(exp) 
q3_2 =  pi - q3_1 %%%%%%%%%%%%%%%%%%%% 
sin(pi - q3_1);
% q3indeg = rad2deg(q3)
% q3 = 0.165555235365465;

%% shouldn't we be getting two points

%% next we find q2 analytically
w_1_anal = [ sin(q2 + q3)* d4 + a2*cos(q2);-cos(q2 + q3)* d4 + a2* sin(q2); 0; 1];
% solution for A sin q2 + B cos q2 = c
A = d4 * cos(q3);
B = a2 + (d4 * sin(q3));
C = o4_1aug(1,1);

r = sqrt(A^2 + B^2);
beta = atan2(B, A);

q2 = asin(C/r) - beta
norm(q2);

%% next we find all the other angles using these result
R6_0 = [ 0.707107, 0 , 0.707107;
            0   ,    -1 , 0      ;
            0.707107 ,0 ,-0.707107];
dh_row(2,:);
%%%%%%%%%%%%%%%%%%%%%%%%%A21 = find_A_from_DH(dh_row(2,:));
A21 = find_A_from_DH(dh_row(2,:));
dh_row(3,:);
A32 = find_A_from_DH(dh_row(3,:));
%we know A10

% taking inverse of each matrix and extracting rotation matrix only

A23 = inv(A32); R23 = A23(1:3,1:3);
A12 = inv(A21); R12 = A12(1:3,1:3);
A01 = inv(A10); R01 = A01(1:3,1:3);

R6_3_des = R23 * R12 * R01 * R6_0;

% from hand calculation sheet
q5 = acos(R6_3_des(3,3))
q6 = atan2(R6_3_des(3,2), R6_3_des(3,1))
q4 = atan2(R6_3_des(2,3), R6_3_des(1,3))







%% forward Kinematic from dh values
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
