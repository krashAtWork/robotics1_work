a_vec = [1, 0, 0];
d_vec = [0, 0, 1];
alpha_vec = [ pi/2, pi/2, 0 ];
theta_vec = [  0, pi/4, 0];

A_ref = fk(a_vec, d_vec, alpha_vec, theta_vec);
calc_jac(A_ref)