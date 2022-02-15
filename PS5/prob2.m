%% Jacobian numerical validation
clc
%% First find w_ref using q_ref

% motor constants or dh values
a_vec = [0.145, 1.150, 0.250, 0, 0, 0];
d_vec = [0.54, 0, 0, -1.812, 0, -0.1];
alpha_vec = [3*pi/2, -pi, -pi/2, pi/2, 3*pi/2, pi];

% given q_ref
q_ref = [0.87674,  -0.78611,   0.21930];
theta_vec = [q_ref, 0, 0, 0];

A_ref = fk(a_vec, d_vec, alpha_vec, theta_vec);
o4_0 = A_ref(17:19, 4);
w = o4_0 %% obtained from prob 1;

%% now if we perturb the motor on joint 1

q_dq1 = q_ref +[0.001,0,0];

% wrist posn shifts
q_dq1 = [q_dq1,0 ,0 ,0 ] % augmenting to use in function
theta_vec = q_dq1;

A_dq1 = fk(a_vec, d_vec, alpha_vec, theta_vec);
w_dq1 = A_dq1(17:19, 4)
err = w_dq1 - w %/ 0.001;
J1c = err/ 0.001 ;

%% now if we perturb the motor on joint 2

q_dq2 = q_ref +[0, 0.001,0];

% wrist posn shifts
q_dq2 = [q_dq2,0 ,0 ,0 ]; % augmenting to use in function
theta_vec = q_dq2;

A_dq2 = fk(a_vec, d_vec, alpha_vec, theta_vec);
w_dq2 = A_dq2(17:19, 4)
J2c = (w_dq2 - w )/ 0.001 ;


%% now if we perturb the motor on joint 3

 q_dq3 = q_ref +[0,0,0.001];
 
% wrist posn shifts
q_dq3 = [q_dq3, 0 ,0 ,0 ] % augmenting to use in function
theta_vec = q_dq3;

A_dq3 = fk(a_vec, d_vec, alpha_vec, theta_vec);
w_dq3 = A_dq3(17:19, 4)
J3c = (w_dq3 - w )/ 0.001 ;

%% jacobian comparision

J_approx = [J1c, J2c, J3c]
J_previous = [ -2.01508, 0.03459, 0.48593 ;
 1.67689, 0.04157, 0.58393 ;
 0.00000, -2.47655, 1.66395 ]



 

