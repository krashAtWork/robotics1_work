%% Jacobian Computation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% confirm the forward kinematic,given the values and compare with solution

% motor constants or dh values
a_vec = [0.145, 1.150, 0.250, 0, 0, 0];
d_vec = [0.54, 0, 0, -1.812, 0, -0.1];
alpha_vec = [3*pi/2, -pi, -pi/2, pi/2, 3*pi/2, pi];

%% solution given
p_tool = [   1.69134; 2.05840; 0.68304 ];
R_tool =[
  0.84807 ,  0.50982 ,  0.14446 ;
 -0.51742 , 0.73797 ,  0.43321 ;
  0.11425 , -0.44214,   0.88964];
A_tool = [R_tool,p_tool; 0,0,0,1];



% motor variable
theta_vec = [0.87674,  -0.78611,   0.21930,   0.16801,   1.68849,   4.65091];

% A6_0 = A_ref[]
A_ref = fk(a_vec, d_vec, alpha_vec, theta_vec) ; 
A6_0 = A_ref(21:24,1:4);

% find wrist point and compare with reference.
w = [   1.67689; 2.01508; 0.59408];
o4_0 = A_ref(17:19, 4);


%% confirmed forward kinematics works
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% matlab computation of Jacobian

% orientation Jacobian Parameters: b^ vectors in 0,1 and 2 frames.
b_cap0 = [0, 0, 1]';
b_cap1 = A_ref(1:3,3); %obtain from R1_0 3rd column
b_cap2 = A_ref(5:7,3); %obtain from R2_0

Jac_orient = [b_cap0, b_cap1, b_cap2];


% positional Jacobian Parametrs:
% required params

o1_0 = A_ref(1:3, 4);% from R1_0 4th column
o2_0 = A_ref(5:7, 4);% from R2_0


J1p = cross(b_cap0 , o4_0); % cross product

J2p = cross(b_cap1 , (o4_0 - o1_0));
J3p = cross(b_cap2 , (o4_0 - o2_0));

Jac_pos = [J1p, J2p, J3p]
Jw_ref = [ -2.01508, 0.03459, 0.48593 ;
 1.67689, 0.04157, 0.58393 ;
 0.00000, -2.47655, 1.66395 ];

%% confirmed 1st answer obtained matches reference














