%% Assignment 3 - q1

%% To find the A matrix for the tool flange

%% A_tf_base = A6/5 * A5/4 * A4/3 * A3/2 *A2/1 * A1/0 ;
clc

q1 = 0.876744;
q2 = 0.784691;
q3 = 0.219304;
q4 = 0.168016;
q5 = 1.68849;
q6 = 4.65091;
q = [q1,q2,q3,q4,q5,q6]' ;

%dh_row = zeros(6,4);
dh_row1 = [0.145, 0.540, 3*pi/2 , q1];
dh_row2 = [1.150, 0.0, -pi , q2-pi/2];
dh_row3 = [0.250, 0.0, -pi/2 , q3];
dh_row4 = [0.0 ,-1.812 ,pi/2 , q4];
dh_row5 = [0.0 ,0 , 3*pi/2 , q5];
dh_row6 = [0.0 ,-0.100 ,pi , q6];
dh_row =[ dh_row1; dh_row2; dh_row3; dh_row4; dh_row5 ; dh_row6];


% test_matrix(dh_row)
% test_matrix (q);








%% to find A1/0 
 A10 = find_A_from_DH(dh_row(1,:));% give entire first row
 A21 = find_A_from_DH(dh_row(2,:));
 A32 = find_A_from_DH(dh_row(3,:));
 A43 = find_A_from_DH(dh_row(4,:));
 A54 = find_A_from_DH(dh_row(5,:));
 A65 = find_A_from_DH(dh_row(6,:));

%% find answer first and then try to find a way to visualise it.


 A60 = A65 * A54 * A43 * A32 *A21 * A10 
 A60 = A10 * A21 * A32 * A43 * A54 * A65
 R_tf_base = A60(1:3,1:3)
 o_tf_base = A60(1:3,4)

%% put function here
function [] = test_matrix(mat)
    size(mat)
    mat 
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
    A(1,2) = -sin(theta)*cos(alpha);
    A(1,3) = sin(theta)* sin(alpha);
    A(1,4) = a* cos(theta);
    % row 2
    A(2,1) = sin(theta);
    A(2,2) = cos(theta)*cos(alpha);
    A(2,3) = -cos(theta)* sin(alpha);
    A(2,4) = a* sin(theta);
    % row 3
    A(3,1) = 0;
    A(3,2) = sin(alpha);
    A(3,3) = cos(alpha);
    A(3,4) = d ;
    % row 4
    A(4,1:3) = 0;  
%    A(4,4) = d ;
    
end





