clc
%% to find the A matrix for the left foot wrt world frame.
%% given : A matrix for the right foot wrt world frame.
%%         and the dh parameters for left and right foot.

%% approach : fnd the A pelvis/ world and use it to find the Aright foot/world

%% gather the dh parameters in a neat fashion

%% right_leg_param

q1 = 0;
q2 = -pi/2;
q3 = 0;
q4 = 0;
q5 = 0;
q6 = 0;
stance_q = [q1,q2,q3,q4,q5,q6]' ;
q_right_leg = [0.1, -0.1, 0.2, 0.2, -0.4, -0.1];
q_left_leg = [0.1,0.2,0.3,0.4,0.5,0.6];



ss_pelvis_rl0 = [0.1, 0 ,pi, -pi/2]; 
rl1 = [0, 0, -pi/2, q_right_leg(1)];
rl2 = [0.05, 0, -pi/2 , q_right_leg(2)];
rl3 = [0.4, 0, pi , q_right_leg(3)];
rl4 = [0.5 ,0, 0, q_right_leg(4)];
rl5 = [0, 0, pi/2, q_right_leg(5)];
rl6 = [0.07, 0, 0, q_right_leg(6)];

dh_rl =[ss_pelvis_rl0;
         rl1; 
         rl2; 
         rl3; 
         rl4;
         rl5 ; 
         rl6];

%% left_leg_param


ss_pelvis_ll0 = [0.1, 0, 0, pi/2]; 
ll1 = [0 ,0, +pi/2, q_left_leg(1)];
ll2 = [0.05, 0, +pi/2 ,q_left_leg(2)];
ll3 = [0.4, 0, pi, q_left_leg(3)];
ll4 = [0.5,0, 0, q_left_leg(4)];
ll5 = [0, 0, -pi/2, q_left_leg(5)];
ll6 = [0.07, 0, pi, q_left_leg(6)];

dh_ll =[ss_pelvis_ll0;
         ll1; 
         ll2; 
         ll3; 
         ll4;
         ll5 ; 
         ll6];

 %% A matrix for the right foot wrt world frame.
 A_rfoot_world = [0, 0, 1, 1;
                  0, 1, 0, 2;
                  0, 0, 1, 3;
                  0, 0, 0, 1];
    
%% A_rl6_static_pelvis calculation


Arl6_world = A_rfoot_world 
Ar10_static_pelvis = find_A_from_DH(dh_rl(1,:));
Arl1_rl0 = find_A_from_DH(dh_rl(2,:));
Arl2_rl1 = find_A_from_DH(dh_rl(3,:));
Arl3_rl2 = find_A_from_DH(dh_rl(4,:));
Arl4_rl3 = find_A_from_DH(dh_rl(5,:));
Arl5_rl4 = find_A_from_DH(dh_rl(6,:));
Arl6_rl5 = find_A_from_DH(dh_rl(7,:));

Arl6_r10 = Arl6_rl5 * Arl5_rl4 * Arl4_rl3 * Arl3_rl2 * Arl2_rl1 * Arl1_rl0 ;

%% should i do r10 wrt pelvis base
Arl6_static_pelvis = Arl6_rl5 * Arl5_rl4 * Arl4_rl3 * Arl3_rl2 * Arl2_rl1 * Arl1_rl0 * Ar10_static_pelvis 

%% A_ll6_static_pelvis calculation
Al10_static_pelvis = find_A_from_DH(dh_ll(1,:));
All1_ll0 = find_A_from_DH(dh_ll(2,:));
All2_ll1 = find_A_from_DH(dh_ll(3,:));
All3_ll2 = find_A_from_DH(dh_ll(4,:));
All4_ll3 = find_A_from_DH(dh_ll(5,:));
All5_ll4 = find_A_from_DH(dh_ll(6,:));
All6_ll5 = find_A_from_DH(dh_ll(7,:));

All6_l10 = All6_ll5 * All5_ll4 * All4_ll3 * All3_ll2 * All2_ll1 * All1_ll0; 
All6_static_pelvis = All6_ll5 * All5_ll4 * All4_ll3 * All3_ll2 * All2_ll1 * All1_ll0 * Al10_static_pelvis 


%% All6/world  calculation
% All6/world =  All6/pelvis * Apelvis/rl6 * Arl6/world
% Apelvis/rl6 = inv(Arl6/pelvis)

Astatic_pelvis_rl6 = inv(Arl6_static_pelvis);
All6_world =  All6_static_pelvis * Astatic_pelvis_rl6 * Arl6_world 

 R_lfoot_world = All6_world(1:3,1:3)
 o_lfoot_world = All6_world(1:3,4)








%%
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
    A(1,4) = a * cos(theta);
    % row 2
    A(2,1) = sin(theta);
    A(2,2) = cos(theta)*cos(alpha);
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



