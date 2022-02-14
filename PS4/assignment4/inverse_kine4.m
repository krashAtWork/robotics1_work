clc

%dh_row = zeros(6,4);
q = sym('q%d', [6 1])
% syms q3 q4 q5; A6_3 = A6_


q(1)
dh_row1 = [0.145, 0.540, 3*pi/2 , q(1)];
dh_row2 = [1.150, 0.0, -pi , q(2)- pi/2];
dh_row3 = [0.250, 0.0, -pi/2 , q(3)];
dh_row4 = [0.0 ,-1.812 ,pi/2 , q(4)];
dh_row5 = [0.0 ,0 , 3*pi/2 , q(5)];
dh_row6 = [0.0 ,-0.100 ,pi , q(6)];
dh_row =[ dh_row1; dh_row2; dh_row3; dh_row4; dh_row5 ; dh_row6];
%%  make a 3d line :
 global inc 
 inc = 40;
i = 0 :1: inc;
x = 2 ;
y = 2 + 0.1*i ;
z = 2 ;
plot3(x,y,z, '-o','Color','b','MarkerSize',1,'MarkerFaceColor','#D9FFFF')


i = 0 :1: 40;
x = 2 ;
y = 2 + 0.1*i ;
z = 2 ;

%% R vector of the to



%% break down the problem into two sub - problems
%% first finding the wrist point ?? How ??
%% what is given ?
   R6_0 = [ 0.707107, 0 , 0.707107;
            0   ,    -1 , 0      ;
            0.707107 ,0 ,-0.707107];
   o6_0i = [2,-2, 0.2]';
   d6 = [0, 0 , -0.1]' ;
   
   A6_0 = [ 0.707107, 0 , 0.707107, 2;
            0   ,    -1 , 0   ,-2   ;
            0.707107 ,0 ,-0.707107, 0.2;
            0, 0 , 0, 1];
   o4_0i = o6_0i -  R6_0  * d6
   %because origin wrt 6 always moves, the wrist position wll also always
   %change
%    o4_0c = A6_0 * d6
   
   
   
   
   

%% use it to find q1, q2, q3 ?? How ?
%% to find q1 
% imagine a rotation about the shoulder axis, which makes a circle about the shoulder point,
% rotating about z axis, only the [x,y] component of the endeffector
% changes.
% tan-1(wy/0, wx/0) = theta 
% if z0 and z1 are coincident then
q1 = atan2(wy/0, wx/0);



%% to find q3, 
% the length of the line between the frame 1 and frame 4 is a function of
% q3 only.
w_1=[sin(q2 + q3) + a2 * cos(q2);
    -cos(q2 + q3) + a2 * sin(q2);
    0;
    ];

mag_w_1 = d4^2 + a2^2 + 2* d4 *a2 * sin(q3)

% we know o4_0i or w_0 numerically; using this we can obtain w_1
% A1_0 = 


% so find 1^ A4
% How ?
%A10 = find_A_from_DH(dh_row(1,:));% give entire first row
A2_1 = find_A_from_DH(dh_row(2,:))
A3_2 = find_A_from_DH(dh_row(3,:))
A4_3 = find_A_from_DH(dh_row(4,:))
% A54 = find_A_from_DH(dh_row(5,:));
% A65 = find_A_from_DH(dh_row(6,:));
A4_1 = A2_1 * A3_2 * A4_3





%% the wrist point is only dependent upon q1, q2, q3.... understand that intuitiviely


q = ik_1(i,x,y,z)
A_obt = fk_q(q)
% process it to find  x,y,z
% compare with above xyz
% maybe a function that finds error

%%  inverse kinematic func 
%% pos -> q values

function q = ik_1(i,x,y,z)
    global inc
  % do something with these pos values and .. dh params
  i , x, y, z
  q = zeros(inc,1,6);
  size(q)
end

%% insert the q obtained above to compare here
function A_obt = fk(q)
    A_flange_base_pose = find_A_flange_base("motoman",q + offset);
end
%% forward kinematic func
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




%% from the wrist point, try to find the pose 1, pose 2, pose3



