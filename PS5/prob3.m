%% Jacobians for numerical inverse kinematics
clc
% motor constants or dh values
a_vec = [0.145, 1.150, 0.250, 0, 0, 0];
d_vec = [0.54, 0, 0, -1.812, 0, -0.1];
alpha_vec = [3*pi/2, -pi, -pi/2, pi/2, 3*pi/2, pi];
    
w_des = [ 1.67689; 2.01508; 0.59408] %% target

% we start from  q_0 = [0;0;0]
% we find w related to it
q_i = [0,0,0];

ei = 1;
N = 100;
for i = 0:N
    
    q_i_aug = [q_i, 0 ,0 ,0 ]; % augmenting to use in function
    theta_vec = q_i_aug;

    A_dqi = fk(a_vec, d_vec, alpha_vec, theta_vec);
    w_dqi = A_dqi(17:19, 4); %% new wrist pos
    
    % error between desired and obtained
    ei = norm(w_des - w_dqi);
    
    if ei < 0.001
        i
        disp("you have reached the position")
        break
    else
        disp("Shifting is required");
    % calculate small positional shift
        dw_i = w_des - w_dqi ;
        Jw_i = calc_jac(A_dqi);
        eps_i = 0.1; %% should get back 0
        dq = eps_i* inv(Jw_i)* dw_i ;
        q_i = q_i + dq'; %% updating q values
    
    end
end
    
    
    



theta_vec = [q_i ,0, 0, 0]
A_ref = fk(a_vec, d_vec, alpha_vec, theta_vec) ; 

o4_0 = A_ref(17:19, 4);
